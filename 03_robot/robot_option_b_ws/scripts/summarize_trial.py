#!/usr/bin/env python3
"""Summarize recorded evidence, without inventing physical measurements."""
import argparse,json
from pathlib import Path

def summarize(rows,gap=None):
    states=[r for r in rows if r.get('event')=='state']
    last=states[-1] if states else {}
    views=[r for r in rows if r.get('event')=='view_result']
    memory=last.get('exploration') or {}
    return dict(task_generation=last.get('task_generation'),target_label=last.get('target_label'),
        terminal_state=last.get('state'),reason=last.get('reason'),
        stationary_view_checks=len(views),viewpoints=memory.get('views',[]),
        excluded=memory.get('excluded',[]),measured_front_gap_m=gap,
        gap_above_course_minimum=(gap>.40 if gap is not None else None),
        result='found' if last.get('state')=='DONE' else 'timeout' if 'search_timeout' in last.get('reason','') else 'incomplete')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('events',type=Path)
    p.add_argument('--measured-gap',type=float);p.add_argument('--output',type=Path)
    args=p.parse_args()
    rows=[json.loads(line) for line in args.events.read_text().splitlines() if line.strip()]
    text=json.dumps(summarize(rows,args.measured_gap),ensure_ascii=False,indent=2)
    if args.output:args.output.write_text(text+'\n')
    else:print(text)
