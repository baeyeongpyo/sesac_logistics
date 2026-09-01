import { readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('.', import.meta.url));
const indexPath = `${root}index.html`;

function inlineSection(name, source) {
  return [
    `<!-- ${name}:START -->`,
    '<script>',
    '(() => {',
    source.trim(),
    '})();',
    '</script>',
    `<!-- ${name}:END -->`,
  ].join('\n');
}

function replaceSection(html, name, source) {
  const section = inlineSection(name, source);
  const marker = new RegExp(`<!-- ${name}:START -->[\\s\\S]*?<!-- ${name}:END -->`);
  if (marker.test(html)) return html.replace(marker, section);
  return html;
}

const [model, app, index] = await Promise.all([
  readFile(`${root}model.mjs`, 'utf8'),
  readFile(`${root}app.mjs`, 'utf8'),
  readFile(indexPath, 'utf8'),
]);

const externalScripts = '    <script src="./model.mjs"></script>\n    <script src="./app.mjs"></script>';
const modelSection = /<!-- NAV2_PATH_MODEL:START -->[\s\S]*?<!-- NAV2_PATH_MODEL:END -->/;
const appSection = /<!-- NAV2_PATH_APP:START -->[\s\S]*?<!-- NAV2_PATH_APP:END -->/;
const hasModelSection = modelSection.test(index);
const hasAppSection = appSection.test(index);

let standalone;
if (hasModelSection && hasAppSection) {
  standalone = replaceSection(index, 'NAV2_PATH_MODEL', model);
  standalone = replaceSection(standalone, 'NAV2_PATH_APP', app);
} else if (index.includes(externalScripts)) {
  standalone = index.replace(
    externalScripts,
    `    ${inlineSection('NAV2_PATH_MODEL', model)}\n    ${inlineSection('NAV2_PATH_APP', app)}`,
  );
} else {
  throw new Error('index.html에서 Nav2Path 스크립트 위치를 찾지 못했습니다.');
}

await writeFile(indexPath, standalone, 'utf8');
