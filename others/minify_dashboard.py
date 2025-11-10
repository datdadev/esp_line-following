import htmlmin
import jsmin
import csscompressor
import re
from pathlib import Path

# THIS SCRIPT IS NOT VALIDATED!
# BRO COULD TEMPORARILY USE THIS INSTEAD https://www.toptal.com/developers/html-minifier

INPUT_HTML = Path("web/index.html")
OUTPUT_HEADER = Path("inc/dashboard_html.h")

html = INPUT_HTML.read_text(encoding="utf-8")

# --- Inline JS/CSS Minification ---
def minify_inline(content):
    # Minify <script>...</script>
    def minify_js(match):
        return f"<script>{jsmin.jsmin(match.group(1))}</script>"

    # Minify <style>...</style>
    def minify_css(match):
        return f"<style>{csscompressor.compress(match.group(1))}</style>"

    content = re.sub(r"<script>([\s\S]*?)</script>", minify_js, content)
    content = re.sub(r"<style>([\s\S]*?)</style>", minify_css, content)
    return content

html = minify_inline(html)

# --- Global HTML Minify ---
minified_html = htmlmin.minify(
    html,
    remove_comments=True,
    remove_empty_space=True,
    remove_all_empty_space=True,
    reduce_boolean_attributes=True,
    remove_optional_attribute_quotes=False
)

# --- Wrap in C Header ---
header_content = f"""#pragma once
#include <pgmspace.h>

const char dashboard_html[] PROGMEM = R"rawliteral(
{minified_html}
)rawliteral";
"""

OUTPUT_HEADER.write_text(header_content, encoding="utf-8")

print(f"✅ Minified {INPUT_HTML.name} → {OUTPUT_HEADER}")
print(f"   Original: {len(html)} bytes")
print(f"   Minified: {len(minified_html)} bytes")
