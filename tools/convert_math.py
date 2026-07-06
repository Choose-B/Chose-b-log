"""
Convert $...$ / $$...$$ to Hexo-compatible {% mathjax %}...{% endmathjax %} format.

Rules:
- $$...$$ -> block-level {% mathjax %}...{% endmathjax %}<br>
- $...$   -> inline {% mathjax %}...{% endmathjax %}<br>
- \\\\ inside a $$ block, when NOT inside any \\begin{...}...\\end{...} environment,
  is replaced by {% endmathjax %}<br>{% mathjax %} to avoid Hexo from rendering \\\\
  as a single backslash.
"""

import re
import sys


def split_math_block(block_content):
    r"""
    Split math block content at unprotected \\\\ line breaks.
    Unprotected means \\\\ NOT inside any \begin{...}...\end{...} environment.
    Returns list of sub-block strings (trimmed).
    """
    lines = block_content.split('\n')
    sub_blocks = []
    current_lines = []
    env_stack = []  # stack of active LaTeX environment names

    for line in lines:
        begins = list(re.finditer(r'\\begin\{([^}]+)\}', line))
        ends = list(re.finditer(r'\\end\{([^}]+)\}', line))

        stripped = line.rstrip()
        has_linebreak = stripped.endswith('\\\\')

        # push begins before checking linebreak,
        # so \\begin{...} on the same line protects the \\
        for b in begins:
            env_stack.append(b.group(1))
        for e in ends:
            env_name = e.group(1)
            if env_stack and env_stack[-1] == env_name:
                env_stack.pop()

        if has_linebreak and not env_stack:
            # unprotected \\\\  - split here
            content_without_break = stripped[:-2].rstrip()
            if content_without_break:
                current_lines.append(content_without_break)
            sub_blocks.append('\n'.join(current_lines).strip())
            current_lines = []
        else:
            current_lines.append(line)

    if current_lines:
        sub_blocks.append('\n'.join(current_lines).strip())

    return [b for b in sub_blocks if b]


def convert_math_tags(filepath):
    """Convert $...$ and $$...$$ to {% mathjax %}...{% endmathjax %}<br>"""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Step 1: Replace $$...$$ block math
    parts = content.split('$$')
    result_parts = []
    for i, part in enumerate(parts):
        if i == 0:
            result_parts.append(part)
        elif i % 2 == 1:
            stripped = part.strip()
            sub_blocks = split_math_block(stripped)
            block_parts = [
                '{% mathjax %}\n' + block + '\n{% endmathjax %}<br>'
                for block in sub_blocks
            ]
            result_parts.append('\n'.join(block_parts))
        else:
            result_parts.append(part)
    content = ''.join(result_parts)

    # Step 2: Replace remaining $...$ inline math (no <br> needed for inline)
    content = re.sub(r'\$(.+?)\$', r'{% mathjax %}\1{% endmathjax %}', content)

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f'Done: {filepath}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 convert_math.py <filepath>')
        sys.exit(1)
    convert_math_tags(sys.argv[1])
