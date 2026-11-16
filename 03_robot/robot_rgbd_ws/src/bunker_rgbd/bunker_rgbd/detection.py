"""The bundled YOLO-World weights use five prompts for the same task target."""
BIN_LABELS=('trash can','garbage bin','waste bin','dustbin','recycling bin')


def is_target_label(label, target_label='trash can'):
    def normalize(value):return ' '.join(value.replace('_',' ').casefold().split())
    return normalize(label) in {normalize(value) for value in (*BIN_LABELS,target_label)}
