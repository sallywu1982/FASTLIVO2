// 将《FAST-Calib 原理与代码逐步解析.md》转换为自包含 HTML（手机友好、公式预渲染、离线可用）
const fs = require('fs');
const path = require('path');
const katex = require('katex');
const { marked } = require('marked');

const ROOT = 'E:/StoneRecord/ws-Work/06_FAST-LIVO2学习/FAST_CALIB';
const SRC = path.join(ROOT, 'FAST-Calib 原理与代码逐步解析.md');
const OUT = path.join(ROOT, 'FAST-Calib 原理与代码逐步解析.html');

let md = fs.readFileSync(SRC, 'utf8');

// ---------- 1. 摘出代码块/行内代码，预渲染数学公式为占位符 ----------
const store = new Map();
let counter = 0;
let texErrors = [];
function stash(html) {
  const key = `ZzMz${(counter++).toString(36)}zZ`;
  store.set(key, html);
  return key;
}
function renderTex(tex, displayMode) {
  try {
    return katex.renderToString(tex.trim(), { displayMode, throwOnError: true, strict: false, trust: true });
  } catch (e) {
    texErrors.push(`${displayMode ? '$$' : '$'}${tex.trim().slice(0, 60)} -> ${e.message}`);
    return katex.renderToString(tex.trim(), { displayMode, throwOnError: false, strict: false });
  }
}

md = md.replace(/^```[\s\S]*?^```/gm, (m) => stash(m));      // 围栏代码块
md = md.replace(/`[^`\n]+`/g, (m) => stash(m));              // 行内代码
md = md.replace(/\$\$([\s\S]+?)\$\$/g, (_, t) => stash(renderTex(t, true)));   // 块级公式
md = md.replace(/\$([^$\n]+?)\$/g, (_, t) => stash(renderTex(t, false)));      // 行内公式

// ---------- 2. Markdown -> HTML（GFM 表格 + GitHub 风格标题锚点） ----------
function githubSlug(html) {
  return html.replace(/<[^>]+>/g, '').trim().toLowerCase()
    .replace(/[^\p{L}\p{N}\s-]/gu, '')
    .replace(/\s/g, '-');
}
marked.use({
  gfm: true, breaks: false,
  renderer: {
    heading(...args) {
      let text, level;
      if (typeof args[0] === 'object') { text = this.parser.parseInline(args[0].tokens); level = args[0].depth; }
      else { text = args[0]; level = args[1]; }
      return `<h${level} id="${githubSlug(text)}">${text}</h${level}>`;
    }
  }
});
let body = marked.parse(md);
body = body.replace(/ZzMz[0-9a-z]+zZ/g, (k) => store.has(k) ? store.get(k) : k);

// ---------- 3. KaTeX CSS + base64 内嵌 woff2 字体（离线可用） ----------
const cssPath = require.resolve('katex/dist/katex.min.css');
let css = fs.readFileSync(cssPath, 'utf8');
css = css.replace(/,?url\(fonts\/[^)]+?\.(?:woff|ttf)\)\s*format\("(?:woff|truetype|type)"\)/g, '');
css = css.replace(/url\(fonts\/([^)]+?\.woff2)\)/g, (_, f) => {
  const b64 = fs.readFileSync(path.join(path.dirname(cssPath), 'fonts', f)).toString('base64');
  return `url(data:font/woff2;base64,${b64})`;
});
if (/url\(fonts\//.test(css)) throw new Error('KaTeX 字体未全部内嵌');

// ---------- 4. 手机友好模板 ----------
const html = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="color-scheme" content="light dark">
<title>FAST-Calib 原理与代码逐步解析</title>
<style>
${css}
:root{--bg:#ffffff;--fg:#1f2328;--fg2:#59636e;--border:#d8dee4;--codebg:#f6f8fa;--quote:#f6f8fa;--link:#0969da;--acc:#1a7f37;}
@media (prefers-color-scheme:dark){:root{--bg:#0d1117;--fg:#e6edf3;--fg2:#9198a1;--border:#30363d;--codebg:#161b22;--quote:#161b22;--link:#58a6ff;--acc:#3fb950;}}
html{-webkit-text-size-adjust:100%;scroll-behavior:smooth;}
body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.8 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;}
main{max-width:800px;margin:0 auto;padding:22px 16px 90px;overflow-wrap:break-word;}
h1,h2,h3,h4{line-height:1.35;font-weight:700;scroll-margin-top:12px;}
h1{font-size:1.5em;margin:1.1em 0 .6em;border-bottom:2px solid var(--border);padding-bottom:.35em;}
h2{font-size:1.28em;margin:1.8em 0 .6em;border-bottom:1px solid var(--border);padding-bottom:.3em;}
h3{font-size:1.12em;margin:1.5em 0 .5em;}
h4{font-size:1em;margin:1.2em 0 .4em;}
p{margin:.7em 0;}
a{color:var(--link);text-decoration:none;}
a:hover{text-decoration:underline;}
blockquote{margin:1em 0;padding:.5em 1em;border-left:4px solid var(--acc);background:var(--quote);border-radius:6px;}
blockquote p{margin:.35em 0;}
code{font-family:ui-monospace,SFMono-Regular,Consolas,"Courier New",monospace;font-size:.88em;background:var(--codebg);padding:.15em .4em;border-radius:4px;word-break:break-word;}
pre{background:var(--codebg);border:1px solid var(--border);border-radius:8px;padding:12px;overflow-x:auto;line-height:1.5;}
pre code{background:none;padding:0;font-size:.82em;white-space:pre;}
table{border-collapse:collapse;margin:1em 0;display:block;overflow-x:auto;max-width:100%;}
th,td{border:1px solid var(--border);padding:6px 10px;text-align:left;vertical-align:top;}
th{background:var(--codebg);white-space:nowrap;}
.katex{font-size:1.05em;}
.katex-display{overflow-x:auto;overflow-y:hidden;padding:4px 2px;margin:1em 0;}
hr{border:none;border-top:1px solid var(--border);margin:2em 0;}
ul,ol{padding-left:1.4em;}
li{margin:.3em 0;}
img{max-width:100%;}
</style>
</head>
<body>
<main>
${body}
</main>
</body>
</html>`;

fs.writeFileSync(OUT, html);
console.log(`written: ${OUT}`);
console.log(`size: ${(fs.statSync(OUT).size / 1024).toFixed(0)} KB`);
console.log(`formulas rendered: ${[...store.values()].filter(v => v.includes('katex-html')).length}`);
console.log(`tex errors: ${texErrors.length}`);
texErrors.slice(0, 10).forEach(e => console.log('  ' + e));
