"""
Reverse {% mathjax %} / {% endmathjax %} back to $ / $$ format.

Rules (reverse of convert_math.py):
- Block-level tags (on their own line) -> $$ $$
- Inline tags -> $ $
- Consecutive blocks split by convert_math.py at \\\\ are merged back into a single
  $$ block joined by \\\\.
- <br> tags added after {% endmathjax %} are stripped.
"""

import re
import sys


def merge_consecutive_math_blocks(content):
    r"""
    Merge consecutive $$ blocks that were split from one original block.
    Only merges when there is at most one newline (no blank line) between blocks.
    """
    parts = content.split('$$')
    result = []
    i = 0
    while i < len(parts):
        if i % 2 == 0:
            # non-math text
            result.append(parts[i])
            i += 1
        else:
            # collect consecutive math blocks
            math_blocks = [parts[i].strip()]
            i += 1
            while (i + 1 < len(parts)
                   and parts[i].strip() == ''
                   and parts[i].count('\n') <= 1):
                math_blocks.append(parts[i + 1].strip())
                i += 2
            if len(math_blocks) == 1:
                result.append('$$\n' + math_blocks[0] + '\n$$')
            else:
                # 2 backslash chars (\\\\ in Python source = two \ chars)
                merged = ' \\\\\n'.join(math_blocks)
                result.append('$$\n' + merged + '\n$$')
    return ''.join(result)


def reverse_convert(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Step 1: Block-level tags (alone on a line, possibly with <br>) -> $$
    content = re.sub(r'^\{\%\s*mathjax\s*\%\}\s*$', '$$', content, flags=re.MULTILINE)
    content = re.sub(r'^\{\%\s*endmathjax\s*\%\}(<br>)?\s*$', '$$', content, flags=re.MULTILINE)

    # Step 2: Inline tags -> $
    content = content.replace('{% mathjax %}', '$')
    content = content.replace('{% endmathjax %}', '$')

    # Step 3: Strip <br> tags added after block-level {% endmathjax %}
    content = re.sub(r'\$\$<br>', '$$', content)

    # Step 4: Merge consecutive $$ blocks that were split at \\\\
    content = merge_consecutive_math_blocks(content)

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Done! Converted {filepath}")


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else 'source/_posts/LinearAlgebra.md'
    reverse_convert(path)
