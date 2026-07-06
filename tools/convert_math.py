import re
import sys

def convert_math_tags(filepath):
    """Convert $...$ and $$...$$ to {% mathjax %}...{% endmathjax %}"""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Step 1: Replace $$...$$ blocks
    parts = content.split('$$')
    result_parts = []
    for i, part in enumerate(parts):
        if i == 0:
            result_parts.append(part)
        elif i % 2 == 1:
            stripped = part.rstrip()
            result_parts.append('{% mathjax %}' + stripped + '\n{% endmathjax %}')
        else:
            result_parts.append(part)
    content = ''.join(result_parts)

    # Step 2: Replace remaining $...$ inline math
    content = re.sub(r'\$(.+?)\$', r'{% mathjax %}\1{% endmathjax %}', content)

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f'Done: {filepath}')

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 convert_math.py <filepath>')
        sys.exit(1)
    convert_math_tags(sys.argv[1])
