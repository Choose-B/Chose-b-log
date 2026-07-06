"""
将 LinearAlgebra.md 中的 {% mathjax %} / {% endmathjax %} 反转回 $ / $$ 格式。

规则（与 CLAUDE.md 中的转换方向相反）：
- 单独成行的 {% mathjax %} / {% endmathjax %} → 块级公式 $$ $$
- 行内与文字混排的 {% mathjax %} / {% endmathjax %} → 行内公式 $ $
"""

import re
import sys

def reverse_convert(filepath: str) -> None:
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Step 1: 处理块级公式 —— {% mathjax %} 单独占一行
    # 匹配行首的 {% mathjax %}（行尾可能有空白）
    content = re.sub(r'^\{\%\s*mathjax\s*\%\}\s*$', '$$', content, flags=re.MULTILINE)
    # 匹配行首的 {% endmathjax %}（行尾可能有空白）
    content = re.sub(r'^\{\%\s*endmathjax\s*\%\}\s*$', '$$', content, flags=re.MULTILINE)

    # Step 2: 处理行内公式 —— 剩余的 {% mathjax %} / {% endmathjax %} 直接替换为 $
    content = content.replace('{% mathjax %}', '$')
    content = content.replace('{% endmathjax %}', '$')

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Done! Converted {filepath}")

if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else 'source/_posts/LinearAlgebra.md'
    reverse_convert(path)
