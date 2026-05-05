"""
平衡车多智能体PID调参系统
架构: Planner(决策) → Tuner(参数) → [apply] → Reflector(反思) → Memory(经验)
同一个 DeepSeek API，三次调用，每次角色不同。
"""
import json, os, re, statistics, requests
from datetime import datetime

MEMORY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuning_memory.json")
REPORT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuning_report.md")
DEEPSEEK_MODEL = "deepseek-v4-flash"
AI_SECS = 5


# ── 持久化记忆库 ──────────────────────────────────────────────
class TuningMemory:
    def __init__(self):
        self.data = self._load()

    def _load(self):
        try:
            with open(MEMORY_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {"attempts": [], "lessons": [], "summary": "", "total": 0}

    def save(self):
        try:
            with open(MEMORY_FILE, "w", encoding="utf-8") as f:
                json.dump(self.data, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def add_attempt(self, attempt: dict):
        self.data["attempts"].append(attempt)
        self.data["total"] = self.data.get("total", 0) + 1
        if len(self.data["attempts"]) > 50:
            self.data["attempts"] = self.data["attempts"][-50:]
        self.save()

    def add_lesson(self, lesson: str):
        if lesson and lesson not in self.data["lessons"]:
            self.data["lessons"].append(lesson)
            if len(self.data["lessons"]) > 20:
                self.data["lessons"] = self.data["lessons"][-20:]
            self.save()

    def set_summary(self, summary: str):
        self.data["summary"] = summary
        self.save()

    def lessons_text(self):
        # 优先用 summary（已凝练），没有再列原始经验条目
        s = self.data.get("summary", "")
        if s:
            return f"【已凝练总结】\n{s}"
        ls = self.data.get("lessons", [])
        return "\n".join(f"- {l}" for l in ls[-10:]) if ls else "暂无历史经验"

    def attempts_text(self, n=6):
        attempts = self.data.get("attempts", [])[-n:]
        if not attempts:
            return "暂无调参记录"
        lines = []
        for a in attempts:
            icon = {"better": "✓", "worse": "✗回滚", "neutral": "→"}.get(a.get("outcome", ""), "?")
            pb = a.get("params_before", {}); pa = a.get("params_after", {})
            lines.append(
                f"[{a.get('ts','')[-8:]}] {a.get('loop','')} "
                f"std:{a.get('std_before',0):.1f}→{a.get('std_after',0):.1f} {icon}  "
                f"Kp:{pb.get('kp',0):.2f}→{pa.get('kp',0):.2f} "
                f"Ki:{pb.get('ki',0):.4f}→{pa.get('ki',0):.4f} "
                f"Kd:{pb.get('kd',0):.3f}→{pa.get('kd',0):.3f}"
            )
        return "\n".join(lines)

    @property
    def lesson_count(self):
        return len(self.data.get("lessons", []))

    @property
    def attempt_count(self):
        return self.data.get("total", 0)


# ── 工具函数 ──────────────────────────────────────────────────
def _stats(pitches):
    mean = statistics.mean(pitches)
    std  = statistics.stdev(pitches) if len(pitches) > 1 else 0
    overshoot = max(abs(min(pitches)), abs(max(pitches)))
    crossings = sum(1 for i in range(1, len(pitches)) if pitches[i-1] * pitches[i] < 0)
    period_ms = int(AI_SECS * 1000 / crossings) if crossings > 1 else 0
    n = len(pitches); seg = max(1, n // AI_SECS)
    early_std = statistics.stdev(pitches[:seg]) if seg > 1 else std
    late_std  = statistics.stdev(pitches[-seg:]) if seg > 1 else std
    trend = "收敛" if late_std < early_std * 0.7 else ("发散" if late_std > early_std * 1.3 else "持续振荡")
    return mean, std, overshoot, crossings, period_ms, trend


def _extract_json(text: str):
    """多策略从文本中提取 JSON dict，尽量不报错"""
    if not text:
        raise ValueError("响应为空")

    # 策略1：直接解析整体
    try:
        return json.loads(text.strip())
    except Exception:
        pass

    # 策略2：找 ```json ... ``` 代码块
    m = re.search(r'```json\s*(\{.*?\})\s*```', text, re.DOTALL)
    if m:
        try: return json.loads(m.group(1))
        except Exception: pass

    # 策略3：找第一个完整 {...}（支持嵌套）
    depth = 0; start = -1
    for i, c in enumerate(text):
        if c == '{':
            if depth == 0: start = i
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0 and start != -1:
                try: return json.loads(text[start:i+1])
                except Exception: start = -1

    raise ValueError(f"无法从响应中提取JSON，原文: {text[:200]}")


def _call_api(api_key, api_url, prompt, log_q, role, max_tokens=1024):
    """调用 DeepSeek，返回解析后的 dict 和原始 content"""
    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    payload = {
        "model": DEEPSEEK_MODEL,
        "messages": [{"role": "user", "content": prompt}],
        "thinking": {"type": "enabled"},
        "reasoning_effort": "medium",
        "stream": False,
        "max_tokens": max_tokens,
    }
    resp = requests.post(api_url, headers=headers, json=payload, timeout=120,
                         proxies={"http": None, "https": None})
    resp.raise_for_status()
    msg = resp.json()["choices"][0]["message"]
    content = (msg.get("content") or "").strip()
    # 兼容 thinking_content（旧字段）和 reasoning_content（DeepSeek-R1 字段）
    thinking = (msg.get("thinking_content") or msg.get("reasoning_content") or "").strip()
    if thinking:
        log_q.put(f"[{role}] 推理: {thinking[:150]}...")
    # 优先取 <think> 外部的内容；若剥离后为空，再从 <think> 内部找；最后 fallback 到 thinking
    outside = re.sub(r'<think>.*?</think>', '', content, flags=re.DOTALL).strip()
    inside_m = re.search(r'<think>(.*?)</think>', content, re.DOTALL)
    inside = inside_m.group(1).strip() if inside_m else ""
    raw = outside or inside or thinking
    if not raw:
        # 打印完整 message 结构，方便排查字段名
        log_q.put(f"[{role}] 响应字段: {list(msg.keys())}  content={content!r:.100}  thinking={thinking!r:.100}")
        raise ValueError(f"响应为空，原始content: {content[:200]!r}")
    return _extract_json(raw), content


def _call_api_text(api_key, api_url, prompt, log_q, role, max_tokens=2048):
    """调用 DeepSeek，直接返回文本（用于报告生成，不需要JSON）"""
    headers = {"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"}
    payload = {
        "model": DEEPSEEK_MODEL,
        "messages": [{"role": "user", "content": prompt}],
        "thinking": {"type": "enabled"},
        "reasoning_effort": "medium",
        "stream": False,
        "max_tokens": max_tokens,
    }
    resp = requests.post(api_url, headers=headers, json=payload, timeout=120,
                         proxies={"http": None, "https": None})
    resp.raise_for_status()
    msg = resp.json()["choices"][0]["message"]
    content = (msg.get("content") or "").strip()
    thinking = (msg.get("thinking_content") or msg.get("reasoning_content") or "").strip()
    if thinking:
        log_q.put(f"[{role}] 推理: {thinking[:150]}...")
    outside = re.sub(r'<think>.*?</think>', '', content, flags=re.DOTALL).strip()
    inside_m = re.search(r'<think>(.*?)</think>', content, re.DOTALL)
    inside = inside_m.group(1).strip() if inside_m else ""
    return outside or inside or thinking


# ── Planner + Tuner 主循环 ────────────────────────────────────
def run_agent_cycle(api_key, api_url, store, memory, log_q, result_q):
    try:
        pitches = store.recent_pitch(AI_SECS)
        if len(pitches) < 10:
            log_q.put("[Agent] 数据不足，等待几秒再试")
            result_q.put(("agent", None)); return

        mean, std, overshoot, crossings, period_ms, trend = _stats(pitches)

        # ── Step 1: Planner ──────────────────────────────────
        planner_prompt = f"""你是两轮自平衡小车调参规划师。根据当前传感器状态和历史经验，决定下一步动作。

## 当前状态
- pitch 均值:{mean:.2f}°  标准差:{std:.2f}°  最大偏差:{overshoot:.2f}°
- 振荡过零:{crossings}次  周期:{period_ms}ms  趋势:{trend}
- 角度环: Kp={store.kp:.3f} Ki={store.ki:.4f} Kd={store.kd:.3f}
- 速度环: Kp={store.spd_kp:.3f} Ki={store.spd_ki:.4f} Kd={store.spd_kd:.3f}
- 控制模式: {"完整双环" if store.mode == 1 else "仅角度环"}

## 历史经验（必须优先参考，不要重蹈覆辙！）
{memory.lessons_text()}

## 最近调参记录
{memory.attempts_text()}

## 决策规则（按优先级）
1. 趋势为"发散" → tune_angle，必须减 Kp（无论什么模式）
2. 控制模式为"仅角度环"：
   - std>4° → tune_angle
   - std 在 1°~4° → tune_angle，重点调 Ki 消稳态误差
   - std<1° 且收敛 → observe
3. 控制模式为"完整双环"：
   - std>5° → tune_angle（角度环太乱，先稳住）
   - std≤5° → tune_speed（双环模式下速度环是主要调整目标，角度环参数不动）
4. 同一方向连续2次调整都变差（见记录✗）→ observe 或换反方向

只返回 JSON（禁止输出任何其他文字）：
{{"action": "tune_angle"|"tune_speed"|"observe", "reason": "一句话理由", "analysis": "当前主要问题"}}"""

        log_q.put("[Agent] Planner 分析中...")
        plan, _ = _call_api(api_key, api_url, planner_prompt, log_q, "Planner")
        action = plan.get("action", "observe")
        log_q.put(f"[Planner] {plan.get('analysis','')} → {action}：{plan.get('reason','')}")

        if action == "observe":
            log_q.put("[Agent] 当前状态良好，本轮不调整")
            result_q.put(("agent", {"action": "observe"})); return

        # ── Step 2: Tuner ────────────────────────────────────
        if action == "tune_angle":
            kp_min = round(store.kp * 0.80, 3); kp_max = round(store.kp * 1.20, 3)
            ki_cur = store.ki
            # Ki 为 0 时保守起步 0~0.1，避免积分饱和；否则 ±30%
            if ki_cur == 0:
                ki_min, ki_max = 0.0, 0.1
            else:
                ki_min = round(ki_cur * 0.70, 4); ki_max = round(ki_cur * 1.30, 4)
            kd_min = round(store.kd * 0.70, 3); kd_max = round(store.kd * 1.30, 3)
            tuner_prompt = f"""你是PID参数调整专家。必须同时给出角度环 Kp、Ki、Kd 三个参数。

## 规划师分析
问题：{plan.get('analysis','')}  方向：{plan.get('reason','')}

## 当前参数与允许范围（三个都必须输出，不能省略）
Kp={store.kp:.3f}（{kp_min}~{kp_max}）  Ki={ki_cur:.4f}（{ki_min}~{ki_max}）  Kd={store.kd:.3f}（{kd_min}~{kd_max}）

## Ki 调整规则（强制执行）
- Ki 的作用：消除稳态误差（小车静止时持续偏离0°的角度偏差）
- pitch 均值偏离={mean:.2f}°，{"偏差明显，Ki 必须 > 0，从 0.01~0.05 开始" if abs(mean) > 0.5 and ki_cur == 0 else ""}{"Ki 已有值，在允许范围内调整" if ki_cur > 0 else ""}
- 注意：Ki 过大会导致积分饱和和低频振荡，每次小幅增加
- 只有趋势为"发散"时才允许 Ki=0；其他情况必须给 Ki 一个正值

## 传感器
std={std:.2f}°  均值={mean:.2f}°  过零{crossings}次  周期{period_ms}ms  趋势:{trend}

## 历史经验（必须遵守，已证明有效的策略优先）
{memory.lessons_text()}

只返回 JSON（禁止其他文字，ki 字段必须存在且 > 0，除非趋势发散）：{{"kp": 数值, "ki": 数值, "kd": 数值, "reason": "一句话"}}"""
        else:
            skp_min = round(store.spd_kp * 0.80, 3); skp_max = round(store.spd_kp * 1.20, 3)
            ski_min = round(store.spd_ki * 0.70, 4); ski_max = round(store.spd_ki * 1.30, 4)
            tuner_prompt = f"""你是PID参数调整专家。给出具体速度环参数（角度环锁定不动）。

## 规划师分析
问题：{plan.get('analysis','')}  方向：{plan.get('reason','')}

## 当前参数与允许范围
速度Kp={store.spd_kp:.3f}（{skp_min}~{skp_max}）  速度Ki={store.spd_ki:.4f}（{ski_min}~{ski_max}）

## 历史经验
{memory.lessons_text()}

只返回 JSON（禁止其他文字）：{{"spd_kp": 数值, "spd_ki": 数值, "spd_kd": {store.spd_kd:.3f}, "reason": "一句话"}}"""

        log_q.put("[Agent] Tuner 生成参数中...")
        params, _ = _call_api(api_key, api_url, tuner_prompt, log_q, "Tuner")
        params["action"] = action
        params["std_before"] = std
        # 角度环调参时补全 ki（若 Tuner 未输出 ki 则保持当前值）
        if action == "tune_angle" and "ki" not in params:
            params["ki"] = store.ki
        log_q.put(f"[Tuner] {params.get('reason','')} | 参数: {params}")
        result_q.put(("agent", params))

    except Exception as e:
        log_q.put(f"[Agent] 错误: {e}")
        result_q.put(("agent", None))


# ── Reflector ─────────────────────────────────────────────────
def run_reflector(api_key, api_url, attempt, memory, log_q):
    try:
        outcome_str = {"better": "改善", "worse": "变差并已回滚", "neutral": "无明显变化"}.get(
            attempt.get("outcome", ""), "未知")
        pb = attempt.get("params_before", {}); pa = attempt.get("params_after", {})

        prompt = f"""你是平衡车调参反思专家。根据本次调参结果提炼一条可复用经验。

## 本次调参
调整环:{attempt.get('loop','')}  结果:{outcome_str}
Kp:{pb.get('kp',0):.3f}→{pa.get('kp',0):.3f}  Ki:{pb.get('ki',0):.4f}→{pa.get('ki',0):.4f}  Kd:{pb.get('kd',0):.3f}→{pa.get('kd',0):.3f}
std:{attempt.get('std_before',0):.2f}°→{attempt.get('std_after',0):.2f}°  均值偏移:{attempt.get('mean_before',0):.2f}°→{attempt.get('mean_after',0):.2f}°  原因:{attempt.get('reason','')}

## 已有经验（避免重复）
{memory.lessons_text()}

提炼一条10~20字经验（如"发散时先减Kp再增Kd效果好"）。重复则is_new=false。
只返回 JSON（禁止其他文字）：{{"lesson": "经验内容", "is_new": true}}"""

        data, _ = _call_api(api_key, api_url, prompt, log_q, "Reflector")
        lesson = data.get("lesson", "")
        if lesson and data.get("is_new", True):
            memory.add_lesson(lesson)
            log_q.put(f"[Reflector] 新经验: {lesson}（共{memory.lesson_count}条）")
        else:
            log_q.put(f"[Reflector] 与已有经验重叠，跳过")
    except Exception as e:
        log_q.put(f"[Reflector] 反思失败: {e}")


# ── Report Generator ──────────────────────────────────────────
def run_generate_report(api_key, api_url, store, memory, log_q, result_q):
    """
    生成调参总结报告：
    1. AI 凝练所有经验 → 更新 memory.summary（下次Planner直接用）
    2. 生成 markdown 报告存到 tuning_report.md
    结果放入 result_q: ("report", report_text | None)
    """
    try:
        lessons = memory.data.get("lessons", [])
        attempts = memory.data.get("attempts", [])
        if not attempts and not lessons:
            log_q.put("[报告] 暂无调参记录，无法生成报告")
            result_q.put(("report", None)); return

        log_q.put("[报告] 正在生成总结报告...")

        # 统计
        total = len(attempts)
        better = sum(1 for a in attempts if a.get("outcome") == "better")
        worse  = sum(1 for a in attempts if a.get("outcome") == "worse")

        # 找最优参数（std最小的better记录）
        best = min((a for a in attempts if a.get("outcome") == "better"),
                   key=lambda a: a.get("std_after", 999), default=None)

        best_text = ""
        if best:
            pa = best.get("params_after", {})
            best_text = f"最优角度环: Kp={pa.get('kp',0):.3f} Ki={pa.get('ki',0):.4f} Kd={pa.get('kd',0):.3f}，std={best.get('std_after',0):.2f}°"

        all_attempts_text = "\n".join([
            f"- [{a.get('ts','')[:16]}] {a.get('loop','')} "
            f"std:{a.get('std_before',0):.1f}→{a.get('std_after',0):.1f} "
            f"{'✓改善' if a.get('outcome')=='better' else ('✗回滚' if a.get('outcome')=='worse' else '→持平')} "
            f"原因:{a.get('reason','')}"
            for a in attempts[-20:]
        ])

        prompt = f"""你是平衡车调参专家。根据以下完整调参历史，生成两部分输出：

## 调参统计
- 总次数:{total}  改善:{better}  回滚:{worse}
- {best_text}
- 当前参数: 角度环 Kp={store.kp:.3f} Ki={store.ki:.4f} Kd={store.kd:.3f}

## 原始经验条目
{chr(10).join(f'- {l}' for l in lessons) if lessons else '暂无'}

## 调参历史记录
{all_attempts_text}

---
请输出两部分，格式严格如下（用===分隔）：

SUMMARY
（3~5句话的凝练总结，供下次调参直接使用，包含：有效的调参方向、危险操作、当前最优参数范围）

===

REPORT
（完整 Markdown 报告，包含：调参过程分析、有效策略、失败教训、推荐起始参数、下次调参建议）"""

        raw = _call_api_text(api_key, api_url, prompt, log_q, "Reporter", max_tokens=2048)

        # 分割 SUMMARY 和 REPORT
        summary = ""
        report_md = raw
        if "===" in raw:
            parts = raw.split("===", 1)
            summary_raw = parts[0].strip()
            report_md   = parts[1].strip()
            # 去掉 SUMMARY 标题行
            summary = re.sub(r'^SUMMARY\s*\n?', '', summary_raw, flags=re.IGNORECASE).strip()
            report_md = re.sub(r'^REPORT\s*\n?', '', report_md, flags=re.IGNORECASE).strip()

        # 保存 summary 到记忆库（下次 Planner 直接用）
        if summary:
            memory.set_summary(summary)
            log_q.put(f"[报告] 经验已凝练入记忆库：{summary[:80]}...")

        # 写 markdown 报告文件
        ts = datetime.now().strftime("%Y-%m-%d %H:%M")
        full_report = f"# 平衡车调参报告\n生成时间: {ts}\n\n{report_md}"
        try:
            with open(REPORT_FILE, "w", encoding="utf-8") as f:
                f.write(full_report)
            log_q.put(f"[报告] 已保存到 {REPORT_FILE}")
        except Exception as e:
            log_q.put(f"[报告] 保存失败: {e}")

        result_q.put(("report", full_report))

    except Exception as e:
        log_q.put(f"[报告] 生成失败: {e}")
        result_q.put(("report", None))
