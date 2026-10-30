'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const state = {datasetId: null, manifest: null, annotations: null, taxonomy: [], points: [], memberships: [], names: new Map(), selected: new Set(), draft: new Set(), undo: [], mode: 'orbit', busy: false, conflict: false, generation: 0, membershipVersion: 0, pendingDataset: null};
  const layerControls = {floor: 'layerFloor', wall: 'layerWall', ceiling: 'layerCeiling', object: 'layerObjects', unassigned: 'layerUnassigned'};
  let accessToken = '';
  try {
    const supplied = new URLSearchParams(location.hash.slice(1)).get('token');
    if (supplied && /^[A-Za-z0-9_-]{16,512}$/.test(supplied)) {
      accessToken = supplied;
      sessionStorage.setItem('atlasAnnotationToken', supplied);
      history.replaceState(null, '', location.pathname + location.search);
    } else accessToken = sessionStorage.getItem('atlasAnnotationToken') || '';
  } catch (_) { /* Loopback access remains available if browser storage is disabled. */ }
  const authHeaders = () => accessToken ? {Authorization: `Bearer ${accessToken}`} : {};
  const categoryColors = {floor: [.30,.78,.82], wall: [.89,.73,.44], ceiling: [.64,.59,.89], chair: [.91,.47,.57], table: [.45,.76,.49], sofa: [.76,.51,.85], cabinet: [.47,.64,.91], door: [.87,.59,.36], window: [.48,.81,.76], other: [.73,.73,.76]};
  const typeNames = {structure: 'Estructura', object: 'Posible objeto', unassigned: 'Sin agrupar / ruido'};
  const statusNames = {reviewed: 'Revisado', ignored: 'Ignorado', pending: 'Pendiente'};
  const number = (n, digits = 0) => Number(n).toLocaleString('es', {maximumFractionDigits: digits});
  const vectorText = values => values.map(v => number(v, 2)).join(' × ');
  const labelName = key => state.taxonomy.find(item => item.key === key)?.name || key || 'Sin etiqueta';
  const annotation = id => state.annotations?.groups?.[id] || {status: 'pending', label: null, instance_id: null, notes: ''};
  const groups = () => state.manifest?.groups || [];
  const selectedGroups = () => groups().filter(group => state.selected.has(group.id));
  const groupName = group => state.names.get(group.id) || 'Grupo';
  function assignGroupNames() {
    const count = {structure: 0, object: 0, unassigned: 0};
    state.names = new Map(groups().map(group => {
      if (group.manual) return [group.id, `Objeto manual ${Number(group.id.match(/\d+$/)?.[0]) || group.id}`];
      count[group.kind] = (count[group.kind] || 0) + 1;
      const prefix = group.kind === 'object' ? 'Objeto' : group.kind === 'unassigned' ? 'Puntos dispersos' : 'Grupo';
      return [group.id, `${prefix} ${count[group.kind]}`];
    }));
  }
  function captureName(item, index) {
    const raw = String(item.capture_id || item.source?.capture_id || item.dataset_id || '');
    const match = raw.match(/(\d{4})(\d{2})(\d{2})[-_](\d{2})(\d{2})(\d{2})?/);
    if (!match) return `Captura ${index + 1}`;
    const [, year, month, day, hour, minute] = match;
    const date = new Date(Date.UTC(Number(year), Number(month) - 1, Number(day)));
    if (date.getUTCMonth() !== Number(month) - 1 || date.getUTCDate() !== Number(day)
        || Number(hour) > 23 || Number(minute) > 59) return `Captura ${index + 1}`;
    return `${date.toLocaleDateString('es', {day: 'numeric', month: 'short', year: 'numeric', timeZone: 'UTC'})} · ${hour}:${minute}`;
  }

  class ApiError extends Error { constructor(message, status, body) { super(message); this.status = status; this.body = body; } }
  async function api(path, payload) {
    const response = await fetch(path, {cache: 'no-store', headers: authHeaders(), ...(payload === undefined ? {} : {method: 'POST', headers: {...authHeaders(), 'Content-Type': 'application/json'}, body: JSON.stringify(payload)})});
    let body;
    try { body = await response.json(); } catch (_) { throw new ApiError('El servidor no devolvió una respuesta válida.', response.status, {}); }
    if (!response.ok) throw new ApiError(body.error || `Error ${response.status}`, response.status, body);
    return body;
  }
  const endpoint = suffix => `/api/datasets/${encodeURIComponent(state.datasetId)}${suffix || ''}`;
  function message(text, kind = '') { $('status').textContent = text; $('status').className = kind; }
  function failure(error) {
    if (error.status === 409) {
      state.conflict = true;
      $('reload').hidden = false;
      const revisionConflict = error.body?.code === 'revision_conflict';
      $('reload').textContent = revisionConflict ? 'Recargar etiquetas guardadas' : 'Volver a comprobar captura y catálogo';
      message(revisionConflict ? 'Las etiquetas cambiaron en otra ventana. Este cambio no se guardó. Recarga las etiquetas antes de continuar.' : error.message || String(error), 'error');
    } else message(error.message || String(error), 'error');
    updateButtons();
  }

  function applyTaxonomy(taxonomy) {
    const ids = new Set(), keys = new Set();
    if (!Array.isArray(taxonomy) || !taxonomy.length) throw new Error('El catálogo de categorías no es válido.');
    for (const item of taxonomy) {
      if (!Number.isInteger(item.id) || item.id < 0 || typeof item.key !== 'string' || !item.key || typeof item.name !== 'string' || !item.name || ids.has(item.id) || keys.has(item.key)) throw new Error('El catálogo contiene categorías inválidas.');
      ids.add(item.id); keys.add(item.key);
    }
    const selected = $('category').value;
    if (selected && !keys.has(selected)) throw new Error('El catálogo recibido no incluye tu categoría seleccionada. Se conservó la selección actual.');
    state.taxonomy = taxonomy;
    $('category').replaceChildren(new Option('Elige una categoría', ''), ...taxonomy.map(item => new Option(item.name, item.key)));
    $('category').value = selected;
  }
  function categoryFeedback(text, error = false) {
    $('categoryFeedback').textContent = text; $('categoryFeedback').className = error ? 'error' : '';
  }
  async function openCategoryForm() {
    if (state.busy) return;
    finishStroke(false);
    $('categoryForm').hidden = false; $('newCategory').setAttribute('aria-expanded', 'true');
    state.busy = true; updateButtons(); categoryFeedback('Actualizando categorías…');
    try {
      const result = await api('/api/categories'); applyTaxonomy(result.taxonomy);
      renderList(); updateColors(); categoryFeedback('Escribe el nombre y pulsa «Crear categoría».');
    } catch (error) { if (error.status === 409) failure(error); categoryFeedback(error.message || 'No se pudo actualizar el catálogo.', true); }
    finally { state.busy = false; updateButtons(); $('categoryName').focus({preventScroll:true}); }
  }
  function closeCategoryForm() {
    if (state.busy) return;
    $('categoryForm').hidden = true; $('newCategory').setAttribute('aria-expanded', 'false');
    $('categoryName').value = ''; categoryFeedback('');
  }
  async function createCategory(event) {
    event.preventDefault();
    if (state.busy || state.conflict) return;
    const name = $('categoryName').value.trim();
    if (!name) { categoryFeedback('Escribe un nombre para la categoría.', true); $('categoryName').focus({preventScroll:true}); return; }
    finishStroke(false); state.busy = true; updateButtons(); categoryFeedback('Guardando la categoría…');
    try {
      const result = await api('/api/categories', {name});
      if (!result.category?.key || !result.taxonomy?.some(item => item.key === result.category.key)) throw new Error('El servidor no confirmó la categoría. Puedes volver a intentarlo con el mismo nombre.');
      applyTaxonomy(result.taxonomy); $('category').value = result.category.key;
      $('categoryName').value = ''; $('categoryForm').hidden = true; $('newCategory').setAttribute('aria-expanded','false');
      renderList(); updateColors();
      message(`${result.created ? 'Categoría creada' : 'La categoría ya existía'}: ${result.category.name}. Seleccionada para el próximo guardado; aún no se etiquetó ningún punto.`, 'success');
    } catch (error) { if (error.status === 409) failure(error); categoryFeedback(error.message || 'No se pudo crear la categoría.', true); }
    finally { state.busy = false; updateButtons(); }
  }

  function categoryColor(key) {
    if (Object.prototype.hasOwnProperty.call(categoryColors, key)) return categoryColors[key];
    // Custom class colors depend only on their stable catalog key, not on
    // scene ordering or which object currently carries the class.
    let hash = 2166136261;
    for (const character of String(key)) { hash ^= character.codePointAt(0); hash = Math.imul(hash, 16777619); }
    const hue = (hash >>> 0) % 360 * Math.PI / 180;
    return [0, 2.094, 4.189].map(offset => .63 + .23 * Math.cos(hue + offset));
  }

  function baseColor(group, index) {
    const entry = annotation(group.id);
    if (entry.status === 'ignored') return [.40,.46,.49];
    if (entry.status === 'reviewed' && entry.label) return categoryColor(entry.label);
    if (group.suggested_label && categoryColors[group.suggested_label]) return categoryColors[group.suggested_label];
    if (group.kind === 'unassigned') return [.43,.50,.55];
    const hue = (index * 137.508 + 18) * Math.PI / 180;
    return [0, 2.094, 4.189].map(offset => .63 + .23 * Math.cos(hue + offset));
  }
  function matches(group) {
    const mode = $('filter').value, entry = annotation(group.id);
    const layer = layerKey(group), kind = ['floor','wall','ceiling'].includes(layer) ? 'structure' : layer;
    if (['structure', 'object', 'unassigned'].includes(mode) && kind !== mode) return false;
    if (['pending', 'reviewed'].includes(mode) && entry.status !== mode) return false;
    const query = $('search').value.trim().toLocaleLowerCase('es');
    return !query || [groupName(group), group.id, typeNames[group.kind], labelName(entry.label), labelName(group.suggested_label), entry.notes].join(' ').toLocaleLowerCase('es').includes(query);
  }
  function layerKey(group) {
    const entry = annotation(group.id), label = entry.status === 'reviewed' ? entry.label : group.suggested_label;
    if (['floor', 'wall', 'ceiling'].includes(label)) return label;
    if (entry.status === 'reviewed') return 'object';
    return group.kind === 'unassigned' ? 'unassigned' : 'object';
  }
  function visible(group) { return $(layerControls[layerKey(group)]).checked && matches(group) && (!$('isolate').checked || state.selected.has(group.id)); }
  function textElement(tag, text, className) { const element = document.createElement(tag); element.textContent = text; if (className) element.className = className; return element; }

  function renderProgress() {
    let reviewed = 0, ignored = 0;
    for (const group of groups()) { const status = annotation(group.id).status; reviewed += status === 'reviewed'; ignored += status === 'ignored'; }
    $('reviewedCount').textContent = reviewed;
    $('ignoredCount').textContent = ignored;
    $('pendingCount').textContent = groups().length - reviewed - ignored;
    $('progress').max = Math.max(1, groups().length);
    $('progress').value = reviewed + ignored;
    $('revision').textContent = `Revisión ${state.annotations?.revision || 0}`;
    $('pointCount').textContent = `${number(state.points.length)} puntos medidos`;
    $('captureSummary').textContent = `${number(groups().filter(g => g.kind === 'object').length)} posibles objetos · ${number(groups().filter(g => g.kind === 'structure').length)} grupos estructurales`;
  }
  function renderList() {
    const list = $('groups'), fragment = document.createDocumentFragment();
    let count = 0;
    for (const [index, group] of groups().entries()) {
      if (!matches(group)) continue;
      count++;
      const entry = annotation(group.id), selected = state.selected.has(group.id);
      const row = document.createElement('div'); row.className = `group-row${selected ? ' selected' : ''}${visible(group) ? '' : ' layer-hidden'}`; row.dataset.groupId = group.id;
      const checkbox = document.createElement('input'); checkbox.type = 'checkbox'; checkbox.checked = selected; checkbox.setAttribute('aria-label', `Seleccionar ${groupName(group)}`);
      checkbox.onchange = () => choose(group.id, true);
      const button = document.createElement('button'); button.type = 'button'; button.setAttribute('aria-pressed', String(selected));
      button.onclick = event => choose(group.id, event.ctrlKey || event.metaKey || event.shiftKey);
      const swatch = document.createElement('span'); swatch.className = 'swatch'; swatch.style.background = `rgb(${baseColor(group, index).map(v => Math.round(255*v)).join(',')})`;
      const copy = document.createElement('span'); copy.className = 'group-copy';
      const title = entry.status === 'reviewed' ? labelName(entry.label) : group.suggested_label ? `${labelName(group.suggested_label)} · propuesta` : typeNames[group.kind];
      copy.append(textElement('strong', `${groupName(group)} · ${title}`), textElement('small', `${number(group.point_count)} puntos · ${statusNames[entry.status] || 'Pendiente'}${visible(group) ? '' : ' · oculto'}`));
      const dot = document.createElement('span'); dot.className = `state-dot ${entry.status}`; dot.title = statusNames[entry.status] || 'Pendiente';
      button.append(swatch, copy, dot); row.append(checkbox, button); fragment.append(row);
    }
    if (!count) fragment.append(textElement('p', 'No hay grupos con estos filtros.', 'empty-list'));
    list.replaceChildren(fragment); $('listCount').textContent = `${count} de ${groups().length} grupos`;
    $('quickGroup').replaceChildren(new Option(state.selected.size > 1 ? `${state.selected.size} grupos seleccionados` : 'Selecciona un grupo', ''),
      ...groups().filter(matches).map(group => new Option(`${groupName(group)} · ${labelName(annotation(group.id).label || group.suggested_label)}`, group.id)));
    $('quickGroup').value = state.selected.size === 1 ? [...state.selected][0] : '';
  }
  function renderSelection(preserveInputs = false) {
    const items = selectedGroups(), count = items.length, detail = $('selectionDetails');
    const previousCategory = $('category').value, previousNotes = $('notes').value, drafting = state.draft.size > 0;
    $('save').hidden = drafting; $('savePoints').hidden = !drafting;
    document.querySelector('.review-actions').hidden = drafting;
    $('removeManual').hidden = drafting || count !== 1 || !items[0]?.manual;
    $('removePanel').hidden = true;
    $('selectionTitle').textContent = count === 1 ? groupName(items[0]) : count ? 'Selección conjunta' : 'Etiqueta del objeto';
    $('selectionCount').hidden = !count; $('selectionCount').textContent = `${count} ${count === 1 ? 'grupo' : 'grupos'}`;
    detail.replaceChildren(); $('proposal').hidden = true; $('acceptProposal').hidden = true;
    if (drafting) {
      $('selectionTitle').textContent = 'Nuevo objeto';
      $('selectionCount').hidden = false; $('selectionCount').textContent = `${number(state.draft.size)} puntos`;
      const bounds = boundsOfIndices(state.draft);
      detail.append(textElement('div', `Extensión X × Y × Z: ${vectorText(sub(bounds[1], bounds[0]))} m`));
      detail.append(textElement('div', 'Borrador sin guardar. Estos puntos formarán una nueva instancia al confirmar.'));
    } else if (!count) {
      detail.textContent = 'Haz clic en la nube o en la lista. Puedes seleccionar varios fragmentos del mismo objeto.';
      $('category').value = ''; $('notes').value = '';
    } else {
      const bounds = combinedBounds(items.map(group => group.bounds_m));
      if (count > 1) detail.append(textElement('div', `${count} grupos · ${number(items.reduce((n, group) => n + group.point_count, 0))} puntos`));
      detail.append(textElement('div', `Extensión X × Y × Z: ${vectorText(sub(bounds[1], bounds[0]))} m`));
      if (count === 1) {
        const entry = annotation(items[0].id);
        detail.append(textElement('div', `${number(items[0].point_count)} puntos · ${statusNames[entry.status]}`));
        if (entry.instance_id != null) detail.append(textElement('div', `Instancia revisada: ${entry.instance_id}`));
        if (items[0].kind === 'unassigned') detail.append(textElement('div', 'Este conjunto contiene puntos sin agrupar; puede mezclar regiones distintas.'));
      }
      const sameLabel = items.every(group => annotation(group.id).label === annotation(items[0].id).label);
      const proposed = items[0].suggested_label;
      const sameProposal = proposed && items.every(group => group.suggested_label === proposed);
      $('category').value = sameLabel && annotation(items[0].id).label ? annotation(items[0].id).label : sameProposal ? proposed : '';
      $('notes').value = count === 1 ? annotation(items[0].id).notes || '' : '';
      if (sameProposal) {
        $('proposal').hidden = false;
        $('proposal').textContent = `Propuesta geométrica: ${labelName(proposed)}. ${count === 1 ? items[0].reason || '' : 'Los grupos comparten la misma propuesta.'} Revisa la nube antes de confirmar.`;
        $('acceptProposal').hidden = false; $('acceptProposal').textContent = `Confirmar ${labelName(proposed).toLocaleLowerCase('es')}`;
      }
    }
    $('save').textContent = count > 1 ? `Etiquetar ${count} grupos juntos` : 'Guardar etiqueta';
    $('mergeHint').hidden = drafting || count < 2;
    if (preserveInputs || drafting) { $('category').value = previousCategory; $('notes').value = previousNotes; }
    renderDraftStatus();
    updateButtons();
  }
  function renderDraftStatus() {
    const count = state.draft.size;
    let shown = 0; for (const i of state.draft) if (visibleGroups[state.memberships[i]]) shown++;
    $('draftCount').textContent = `${number(count)} puntos`;
    $('draftHelp').textContent = count ? `${number(count)} puntos seleccionados${shown < count ? ` (${number(count - shown)} ocultos por la vista)` : ''}. ${count > 100000 ? 'Máximo 100.000 puntos por objeto; borra parte de la selección.' : 'Elige una categoría y pulsa «Crear objeto etiquetado».'}` : 'Sólo se seleccionan puntos visibles. Pinta, revisa y guarda una categoría.';
  }
  function updateButtons() {
    const active = !!state.manifest && !state.busy && !state.conflict, selected = state.selected.size > 0, drafting = state.draft.size > 0;
    $('save').disabled = !active || !selected || !$('category').value;
    for (const id of ['ignore', 'reset', 'acceptProposal']) $(id).disabled = !active || !selected || drafting;
    for (const id of ['category', 'notes']) $(id).disabled = !state.manifest || state.busy || (!selected && !drafting);
    $('savePoints').disabled = !active || !drafting || state.draft.size > 100000 || !$('category').value;
    $('focus').disabled = (!selected && !drafting) || state.busy;
    $('clearSelection').disabled = !selected || state.busy;
    $('export').disabled = !active || drafting;
    $('undoStroke').disabled = state.busy || !state.undo.length;
    $('clearDraft').disabled = state.busy || !drafting;
    for (const id of ['removeManual','confirmRemove']) $(id).disabled = !active || drafting;
    for (const id of ['orbitMode','brushMode','eraseBrush','brushRadius']) $(id).disabled = !state.manifest || state.busy;
    $('dataset').disabled = state.busy;
    $('newCategory').disabled = !state.manifest || state.busy;
    $('categoryName').disabled = state.busy;
    $('createCategory').disabled = state.busy || state.conflict || !$('categoryName').value.trim();
    $('cancelCategory').disabled = state.busy;
    $('discardAndSwitch').disabled = state.busy; $('keepDraft').disabled = state.busy;
    $('reload').disabled = state.busy;
    $('quickGroup').disabled = !state.manifest || state.busy;
    for (const control of $('groups').querySelectorAll('input,button')) control.disabled = state.busy;
  }
  function choose(id, additive = false) {
    if (state.busy) return;
    finishStroke(true);
    if (additive) { if (state.selected.has(id)) state.selected.delete(id); else state.selected.add(id); }
    else state.selected = new Set([id]);
    renderList(); renderSelection(!!state.draft.size); updateColors(); requestRender();
  }
  function resetSelection() { finishStroke(true); state.selected.clear(); renderList(); renderSelection(); updateColors(); requestRender(); }
  async function save(action, suppliedLabel) {
    if (state.busy || state.conflict) return;
    finishStroke(true);
    if (state.draft.size || !state.selected.size) return;
    const items = selectedGroups(), payload = {revision: state.annotations.revision, group_ids: items.map(group => group.id), action, notes: $('notes').value};
    if (action === 'label') { payload.label = suppliedLabel || $('category').value; if (!payload.label) return; }
    state.busy = true; updateButtons(); message('Guardando en la captura…');
    try {
      state.annotations = await api(endpoint('/annotations'), payload);
      $('downloads').replaceChildren();
      renderProgress(); renderList(); renderSelection(); updateColors();
      message(action === 'label' ? `Etiqueta guardada: ${labelName(payload.label)}. ${items.length > 1 ? 'Los grupos comparten una instancia.' : 'Revisión humana registrada.'}` : action === 'ignore' ? 'Selección ignorada. Sus puntos no se usarán como ejemplos etiquetados.' : 'Selección pendiente. Se retiró su etiqueta de entrenamiento.', 'success');
    } catch (error) { failure(error); }
    finally { state.busy = false; updateButtons(); requestRender(); }
  }
  async function exportDataset() {
    if (state.busy || state.conflict || state.draft.size) return;
    state.busy = true; updateButtons(); message('Preparando los datos revisados…');
    try {
      const result = await api(endpoint('/export'), {revision: state.annotations.revision});
      const box = $('downloads'); box.replaceChildren();
      box.append(textElement('p', `${number(result.summary?.reviewed_points || 0)} puntos etiquetados · ${number(result.summary?.pending_points || 0)} pendientes. Las propuestas sin confirmar no son etiquetas de entrenamiento.`));
      for (const file of result.files || []) {
        const url = new URL(file.url, location.href);
        if (url.origin !== location.origin || !url.pathname.startsWith(endpoint('/export/'))) continue;
        const link = document.createElement('a'); link.href = url.href; link.download = file.name; link.textContent = `Descargar ${file.name}`;
        link.onclick = async event => {
          event.preventDefault();
          try {
            const response = await fetch(url.href, {cache: 'no-store', headers: authHeaders()});
            if (!response.ok) throw new Error('No se pudo descargar el archivo. Comprueba la conexión con la Mac.');
            const objectUrl = URL.createObjectURL(await response.blob()), download = document.createElement('a');
            download.href = objectUrl; download.download = file.name; document.body.append(download); download.click(); download.remove();
            setTimeout(() => URL.revokeObjectURL(objectUrl), 10000);
          } catch (error) { failure(error); }
        };
        box.append(link);
      }
      message('Exportación guardada en la captura. Esto prepara ejemplos; todavía no entrena una red.', 'success');
    } catch (error) { failure(error); }
    finally { state.busy = false; updateButtons(); }
  }
  function renderWarnings() {
    const warnings = state.manifest.diagnostics?.warnings;
    const entries = Array.isArray(warnings) ? warnings : [];
    $('warnings').replaceChildren(...entries.map(item => textElement('li', typeof item === 'string' ? item : JSON.stringify(item))));
    $('diagnostics').hidden = !entries.length;
  }
  function applyDescription(data) {
    if (!Array.isArray(data.manifest?.groups) || !Array.isArray(data.point_group_ids) || data.point_group_ids.length !== state.points.length || !data.annotations?.groups || !Array.isArray(data.taxonomy)) throw new Error('Los grupos efectivos de la captura no tienen un formato válido.');
    for (const index of data.point_group_ids) if (!Number.isInteger(index) || index < 0 || index >= data.manifest.groups.length) throw new Error('La captura contiene una pertenencia de puntos inválida.');
    finishStroke(true);
    applyTaxonomy(data.taxonomy);
    state.manifest = data.manifest; state.annotations = data.annotations;
    state.memberships = data.point_group_ids; state.membershipVersion++;
    const ids = new Set(groups().map(group => group.id)); state.selected = new Set([...state.selected].filter(id => ids.has(id)));
    assignGroupNames();
    visibilityCache = null;
  }
  async function savePointObject() {
    if (state.busy || state.conflict || !state.draft.size || state.draft.size > 100000 || !$('category').value) return;
    finishStroke(false);
    const indices = [...state.draft].sort((a,b) => a-b), label = $('category').value;
    state.busy = true; updateButtons(); message('Guardando el nuevo objeto…');
    try {
      const data = await api(endpoint('/point-groups'), {revision: state.annotations.revision, action: 'create', point_indices: indices, label, notes: $('notes').value});
      applyDescription(data);
      state.draft.clear(); state.undo = []; state.selected = new Set([groups()[state.memberships[indices[0]]].id]);
      state.pendingDataset = null; $('discardPanel').hidden = true;
      $('downloads').replaceChildren(); renderProgress(); renderList(); updateColors(); renderSelection();
      message(`Objeto guardado: ${labelName(label)}, ${number(indices.length)} puntos medidos. Nueva instancia revisada.`, 'success');
    } catch (error) { failure(error); }
    finally { state.busy = false; updateButtons(); requestRender(); }
  }
  async function removeManualObject() {
    const items = selectedGroups();
    if (state.busy || state.conflict || state.draft.size || items.length !== 1 || !items[0].manual) return;
    state.busy = true; updateButtons();
    try {
      applyDescription(await api(endpoint('/point-groups'), {revision: state.annotations.revision, action: 'remove', group_id: items[0].id}));
      state.selected.clear(); $('downloads').replaceChildren(); renderProgress(); renderList(); updateColors(); renderSelection();
      message('Objeto manual deshecho. Sus puntos regresaron a los grupos originales.', 'success');
    } catch (error) { failure(error); }
    finally { state.busy = false; updateButtons(); requestRender(); }
  }
  async function loadDataset(id) {
    finishStroke(true); pointers.clear();
    const generation = ++state.generation; state.busy = true; state.datasetId = id; state.conflict = false;
    $('loading').hidden = false; $('sceneError').hidden = !!pointProgram; $('reload').hidden = true; $('downloads').replaceChildren(); updateButtons();
    message('Cargando los grupos y sus etiquetas…');
    try {
      const [data, cloud] = await Promise.all([api(endpoint()), api(endpoint('/points'))]);
      if (generation !== state.generation) return;
      if (!Array.isArray(cloud.points)) throw new Error('Los puntos no tienen un formato válido.');
      for (let i = 0; i < cloud.points.length; i++) {
        if (!Array.isArray(cloud.points[i]) || cloud.points[i].length !== 3 || !cloud.points[i].every(Number.isFinite)) throw new Error('La captura contiene coordenadas inválidas.');
      }
      state.points = cloud.points; state.selected.clear(); state.draft.clear(); state.undo = []; applyDescription(data);
      $('filter').value = 'all'; $('search').value = ''; $('isolate').checked = false;
      for (const [key, id] of Object.entries(layerControls)) $(id).checked = key !== 'ceiling';
      $('category').value = ''; $('notes').value = '';
      uploadScene(); fitCore(); renderProgress(); renderList(); renderSelection(); renderWarnings();
      message('Captura lista, techo oculto. Selecciona un grupo o pinta puntos para crear un objeto.');
    } catch (error) {
      if (generation !== state.generation) return;
      state.manifest = null; state.points = []; state.memberships = []; state.selected.clear(); state.draft.clear(); state.undo = [];
      renderList(); renderProgress(); renderSelection();
      $('sceneError').textContent = error.message; $('sceneError').hidden = false; failure(error);
    } finally {
      if (generation === state.generation) { state.busy = false; $('loading').hidden = true; updateButtons(); requestRender(); }
    }
  }
  async function reloadAnnotations() {
    finishStroke(true);
    state.busy = true; updateButtons();
    try {
      applyDescription(await api(endpoint())); state.conflict = false; $('reload').hidden = true;
      renderProgress(); renderList(); updateColors(); renderSelection(true); message(`Etiquetas actualizadas.${state.draft.size ? ' Tu borrador de puntos se conservó.' : ''} Revisa la selección antes de volver a guardar.`);
    } catch (error) { failure(error); }
    finally { state.busy = false; updateButtons(); requestRender(); }
  }

  // Rendering uses the measured coordinates, without interpolating new surfaces.
  const canvas = $('cloud'), gl = canvas.getContext('webgl', {antialias: true, alpha: false, preserveDrawingBuffer: true});
  const add = (a,b) => a.map((x,i) => x+b[i]), sub = (a,b) => a.map((x,i) => x-b[i]);
  const mul = (a,k) => a.map(x => x*k), dot = (a,b) => a.reduce((n,x,i) => n+x*b[i],0);
  const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const norm = a => Math.hypot(...a), unit = a => mul(a,1/(norm(a)||1));
  function combinedBounds(bounds) {
    const minimum = [Infinity,Infinity,Infinity], maximum = [-Infinity,-Infinity,-Infinity];
    for (const box of bounds) for (let k=0;k<3;k++) { minimum[k] = Math.min(minimum[k],box[0][k]); maximum[k] = Math.max(maximum[k],box[1][k]); }
    return bounds.length ? [minimum, maximum] : [[-1,-1,-1],[1,1,1]];
  }
  function boundsOfIndices(indices) {
    const minimum = [Infinity,Infinity,Infinity], maximum = [-Infinity,-Infinity,-Infinity]; let count = 0;
    for (const index of indices) { const point = state.points[index]; if (!point) continue; count++; for (let k=0;k<3;k++) {minimum[k]=Math.min(minimum[k],point[k]);maximum[k]=Math.max(maximum[k],point[k]);} }
    return count ? [minimum,maximum] : [[-1,-1,-1],[1,1,1]];
  }
  function multiply(a,b) { const r = new Float32Array(16); for(let col=0;col<4;col++)for(let row=0;row<4;row++)for(let k=0;k<4;k++)r[col*4+row]+=a[k*4+row]*b[col*4+k]; return r; }
  const camera = {center:[0,0,0],az:-1.1,el:.55,distance:6,radius:2,sceneRadius:2};
  let pointProgram, positionBuffer, colorBuffer, lineBuffer, locations, sceneBounds, coreBounds, vp, right, up, eye, scheduled=false;
  let visibleGroups=[], visibilityCache=null, visibilityVersion=0;
  function compile(type, source) { const shader=gl.createShader(type); gl.shaderSource(shader,source);gl.compileShader(shader);if(!gl.getShaderParameter(shader,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(shader)); return shader; }
  function initializeRenderer() {
    if (!gl) throw new Error('El navegador no pudo iniciar WebGL. La lista de grupos sigue disponible para revisar las etiquetas.');
    pointProgram=gl.createProgram();
    const vertex=compile(gl.VERTEX_SHADER,'attribute vec3 p;attribute vec4 c;uniform mat4 vp;uniform float size;varying vec4 color;void main(){gl_Position=vp*vec4(p,1.);gl_PointSize=size;color=c;}');
    const fragment=compile(gl.FRAGMENT_SHADER,'precision mediump float;varying vec4 color;uniform float roundPoint;void main(){if(color.a<0.1)discard;if(roundPoint>0.5&&length(gl_PointCoord-vec2(0.5))>0.5)discard;gl_FragColor=vec4(color.rgb,1.);}');
    gl.attachShader(pointProgram,vertex);gl.attachShader(pointProgram,fragment);gl.linkProgram(pointProgram);gl.deleteShader(vertex);gl.deleteShader(fragment);
    if(!gl.getProgramParameter(pointProgram,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(pointProgram));
    locations={p:gl.getAttribLocation(pointProgram,'p'),c:gl.getAttribLocation(pointProgram,'c'),vp:gl.getUniformLocation(pointProgram,'vp'),size:gl.getUniformLocation(pointProgram,'size'),round:gl.getUniformLocation(pointProgram,'roundPoint')};
    positionBuffer=gl.createBuffer();colorBuffer=gl.createBuffer();lineBuffer=gl.createBuffer();
  }
  function uploadScene() {
    if (!pointProgram) return;
    const positions=new Float32Array(state.points.length*3), minimum=[Infinity,Infinity,Infinity], maximum=[-Infinity,-Infinity,-Infinity];
    state.points.forEach((point,index)=>{positions.set(point,index*3);for(let k=0;k<3;k++){minimum[k]=Math.min(minimum[k],point[k]);maximum[k]=Math.max(maximum[k],point[k]);}});
    sceneBounds=state.points.length?[minimum,maximum]:[[-1,-1,-1],[1,1,1]];camera.sceneRadius=Math.max(.2,norm(sub(sceneBounds[1],sceneBounds[0]))/2);
    // Frame the measured bulk initially; extreme returns stay in the GPU
    // buffer and remain reachable through pan/zoom or "Ver toda la captura".
    const lower=[], upper=[];
    for(let axis=0;axis<3;axis++) {
      const values=Float64Array.from(state.points,point=>point[axis]).sort();
      lower.push(values.length ? values[Math.floor((values.length-1)*.01)] : -1);
      upper.push(values.length ? values[Math.ceil((values.length-1)*.99)] : 1);
    }
    coreBounds=[lower,upper];
    gl.bindBuffer(gl.ARRAY_BUFFER,positionBuffer);gl.bufferData(gl.ARRAY_BUFFER,positions,gl.STATIC_DRAW);updateColors();
  }
  function updateColors() {
    if (!state.manifest) return;
    const colors=groups().map((group,index)=>{const selected=state.selected.has(group.id), color=baseColor(group,index);return [...color.map(v=>selected?Math.min(1,v*.8+.25):state.selected.size?v*.64:v),visible(group)?1:0];});
    const nextVisibility=groups().map(visible);
    if(nextVisibility.length!==visibleGroups.length||nextVisibility.some((value,index)=>value!==visibleGroups[index])){visibilityCache=null;visibilityVersion++;}
    visibleGroups=nextVisibility;renderDraftStatus();
    if (!pointProgram) return;
    const vertices=new Float32Array(state.points.length*4);state.memberships.forEach((index,i)=>vertices.set(state.draft.has(i)?[1,.91,.27,colors[index][3]]:colors[index],i*4));
    gl.bindBuffer(gl.ARRAY_BUFFER,colorBuffer);gl.bufferData(gl.ARRAY_BUFFER,vertices,gl.DYNAMIC_DRAW);requestRender();
  }
  function fitBounds(bounds) {
    finishStroke(true);
    camera.center=mul(add(bounds[0],bounds[1]),.5);camera.radius=Math.max(.15,norm(sub(bounds[1],bounds[0]))/2);
    const aspect=Math.max(1,canvas.clientWidth)/Math.max(1,canvas.clientHeight);
    const halfFov=Math.min(Math.PI/8,Math.atan(Math.tan(Math.PI/8)*aspect));
    camera.distance=camera.radius/Math.sin(halfFov)*1.05;requestRender();
  }
  function visibleIndices(){return state.memberships.map((group,index)=>visibleGroups[group]?index:-1).filter(index=>index>=0);}
  function fitCore(){
    const indices=visibleIndices();if(!indices.length){message('Activa una capa o cambia los filtros para encuadrar.');return;}
    const lower=[],upper=[];
    for(let axis=0;axis<3;axis++){const values=Float64Array.from(indices,index=>state.points[index][axis]).sort();lower.push(values[Math.floor((values.length-1)*.01)]);upper.push(values[Math.ceil((values.length-1)*.99)]);}
    coreBounds=[lower,upper];camera.az=-1.1;camera.el=.55;fitBounds(coreBounds);
  }
  function fitAll(){const indices=visibleIndices();if(indices.length){camera.az=-1.1;camera.el=.55;fitBounds(boundsOfIndices(indices));}else message('No hay puntos visibles para encuadrar.');}
  function focusSelection(){
    if(state.draft.size){const indices=[...state.draft].filter(index=>visibleGroups[state.memberships[index]]);if(indices.length)fitBounds(boundsOfIndices(indices));else message('La selección está en capas ocultas.');}
    else{const items=selectedGroups().filter(visible);if(items.length)fitBounds(combinedBounds(items.map(group=>group.bounds_m)));else message('La selección está en capas ocultas.');}
  }
  function zoom(factor){finishStroke(true);camera.distance=Math.max(.03,Math.min(1e5,camera.distance*factor));requestRender();}
  function pan(dx,dy){
    if(!right||!up)return;
    const factor=2*camera.distance*Math.tan(Math.PI/8)/Math.max(1,canvas.clientHeight);
    camera.center=add(camera.center,add(mul(right,-dx*factor),mul(up,dy*factor)));
  }
  function projection() {
    eye=add(camera.center,[camera.distance*Math.cos(camera.el)*Math.cos(camera.az),camera.distance*Math.cos(camera.el)*Math.sin(camera.az),camera.distance*Math.sin(camera.el)]);
    const z=unit(sub(eye,camera.center));right=unit(cross([0,0,1],z));up=cross(z,right);
    const view=new Float32Array([right[0],up[0],z[0],0,right[1],up[1],z[1],0,right[2],up[2],z[2],0,-dot(right,eye),-dot(up,eye),-dot(z,eye),1]);
    const aspect=Math.max(1,canvas.width)/Math.max(1,canvas.height), f=1/Math.tan(Math.PI/8), near=Math.max(camera.radius*.001,.001), far=Math.max(camera.distance+camera.sceneRadius*5,near+10);
    return multiply(new Float32Array([f/aspect,0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0]),view);
  }
  function attribute(buffer,location,count){gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.enableVertexAttribArray(location);gl.vertexAttribPointer(location,count,gl.FLOAT,false,0,0);}
  function drawSelectionBoxes() {
    if(state.draft.size)return;
    const vertices=[];for(const group of selectedGroups()) {if(!visible(group))continue;const [a,b]=group.bounds_m;const corners=[[a[0],a[1],a[2]],[b[0],a[1],a[2]],[b[0],b[1],a[2]],[a[0],b[1],a[2]],[a[0],a[1],b[2]],[b[0],a[1],b[2]],[b[0],b[1],b[2]],[a[0],b[1],b[2]]];for(const [i,j]of[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]])vertices.push(...corners[i],...corners[j]);}
    if(!vertices.length)return;gl.bindBuffer(gl.ARRAY_BUFFER,lineBuffer);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(vertices),gl.DYNAMIC_DRAW);attribute(lineBuffer,locations.p,3);
    gl.disableVertexAttribArray(locations.c);gl.vertexAttrib4f(locations.c,.67,.96,.94,1);gl.uniform1f(locations.round,0);gl.disable(gl.DEPTH_TEST);gl.drawArrays(gl.LINES,0,vertices.length/3);
  }
  function render() {
    scheduled=false;if(!pointProgram)return;
    const dpr=Math.min(window.devicePixelRatio||1,2),width=Math.max(1,Math.round(canvas.clientWidth*dpr)),height=Math.max(1,Math.round(canvas.clientHeight*dpr));
    if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height;}
    gl.viewport(0,0,width,height);gl.clearColor(.064,.105,.14,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.useProgram(pointProgram);
    vp=projection();gl.uniformMatrix4fv(locations.vp,false,vp);gl.uniform1f(locations.size,Math.max(2.5,3.4*dpr));gl.uniform1f(locations.round,1);
    if(state.manifest&&state.points.length){attribute(positionBuffer,locations.p,3);attribute(colorBuffer,locations.c,4);gl.drawArrays(gl.POINTS,0,state.points.length);drawSelectionBoxes();}
  }
  function requestRender(){if(!scheduled){scheduled=true;requestAnimationFrame(render);}}
  function pointSize(){return Math.max(2.5,3.4*Math.min(window.devicePixelRatio||1,2));}
  function getVisibility() {
    if(!pointProgram||!state.manifest||!window.AtlasPointSelection)return null;
    render();
    const key=`${state.membershipVersion}:${canvas.width}:${canvas.height}:${pointSize()}:${Array.from(vp).join(',')}`;
    if(!visibilityCache||visibilityCache.key!==key)visibilityCache={key,buffer:AtlasPointSelection.buildVisibility({points:state.points,memberships:state.memberships,visibleGroups,vp,width:canvas.width,height:canvas.height,pointSize:pointSize()})};
    return visibilityCache.buffer;
  }
  function canvasPoint(event){const rect=canvas.getBoundingClientRect();return {x:(event.clientX-rect.left)*canvas.width/rect.width,y:(event.clientY-rect.top)*canvas.height/rect.height};}
  function pick(clientX,clientY,additive,touch=false) {
    if(state.busy)return;const visibility=getVisibility();if(!visibility)return;
    const point=canvasPoint({clientX,clientY}),scale=canvas.width/canvas.clientWidth,radius=(touch?12:5)*scale;
    let hit=-1,nearest=Infinity;
    for(let y=Math.max(0,Math.floor(point.y-radius));y<=Math.min(canvas.height-1,Math.ceil(point.y+radius));y++)for(let x=Math.max(0,Math.floor(point.x-radius));x<=Math.min(canvas.width-1,Math.ceil(point.x+radius));x++){
      const distance=(x+.5-point.x)**2+(y+.5-point.y)**2,index=visibility.pointIndices[y*canvas.width+x];
      if(index>=0&&distance<=radius*radius&&distance<nearest){nearest=distance;hit=index;}
    }
    if(hit>=0)choose(groups()[state.memberships[hit]].id,additive);
  }
  const pointers=new Map();let dragDistance=0,pinchDistance=0,pinchCenter=null,pointerButton=0,hadMultiplePointers=false,stroke=null;
  function refreshDraft(){updateColors();renderSelection(true);requestRender();}
  function paintSegment(event, refresh = true){
    if(!stroke||event.pointerId!==stroke.id)return;
    if(state.busy||stroke.generation!==state.generation||stroke.membershipVersion!==state.membershipVersion||stroke.visibilityVersion!==visibilityVersion){finishStroke(true);return;}
    const previousSize=state.draft.size;
    const current=canvasPoint(event),indices=AtlasPointSelection.collectStroke(stroke.visibility,stroke.previous,current,stroke.radius);
    for(const index of indices){
      if(stroke.erase){if(state.draft.delete(index)){if(!stroke.added.delete(index))stroke.removed.add(index);}}
      else if(!state.draft.has(index)){state.draft.add(index);if(!stroke.removed.delete(index))stroke.added.add(index);}
    }
    stroke.previous=current;
    const changed=previousSize!==state.draft.size;if(refresh&&changed)refreshDraft();return changed;
  }
  function finishStroke(cancelled){
    if(!stroke)return;
    if(cancelled){for(const index of stroke.added)state.draft.delete(index);for(const index of stroke.removed)state.draft.add(index);}
    else if(stroke.added.size||stroke.removed.size){state.undo.push({added:stroke.added,removed:stroke.removed});if(state.undo.length>50)state.undo.shift();}
    stroke=null;refreshDraft();
  }
  function clearDraft(){finishStroke(true);state.draft.clear();state.undo=[];state.pendingDataset=null;$('discardPanel').hidden=true;refreshDraft();message('Selección de puntos cancelada. No se modificaron etiquetas guardadas.');}
  function undoStroke(){
    if(stroke){finishStroke(true);return;}
    const change=state.undo.pop();if(!change)return;
    for(const index of change.added)state.draft.delete(index);for(const index of change.removed)state.draft.add(index);refreshDraft();
  }
  function setMode(mode){finishStroke(true);state.mode=mode;$('orbitMode').classList.toggle('active',mode==='orbit');$('brushMode').classList.toggle('active',mode==='brush');$('orbitMode').setAttribute('aria-pressed',String(mode==='orbit'));$('brushMode').setAttribute('aria-pressed',String(mode==='brush'));canvas.classList.toggle('painting',mode==='brush');}
  function pairGesture(){const[a,b]=[...pointers.values()];return {distance:Math.hypot(a[0]-b[0],a[1]-b[1]),center:[(a[0]+b[0])/2,(a[1]+b[1])/2]};}
  canvas.oncontextmenu=event=>event.preventDefault();
  canvas.onpointerdown=event=>{
    if(state.busy||!state.manifest)return;
    if(stroke&&event.pointerId!==stroke.id)return; // Ignore palm contacts while Pencil owns a stroke.
    const paint=event.pointerType==='pen'||(state.mode==='brush'&&event.button===0);
    if(paint){
      if(pointers.size||state.conflict)return;
      try{
        const visibility=getVisibility();if(!visibility)return;
        canvas.setPointerCapture(event.pointerId);
        stroke={id:event.pointerId,generation:state.generation,membershipVersion:state.membershipVersion,visibilityVersion,previous:canvasPoint(event),visibility,radius:Number($('brushRadius').value)*canvas.width/canvas.clientWidth,erase:$('eraseBrush').checked||event.button===5||(event.buttons&32)!==0,added:new Set(),removed:new Set()};
        paintSegment(event);
      }catch(error){failure(error);}
      return;
    }
    canvas.setPointerCapture(event.pointerId);pointers.set(event.pointerId,[event.clientX,event.clientY]);
    if(pointers.size===1){dragDistance=0;pinchDistance=0;pinchCenter=null;pointerButton=event.button;hadMultiplePointers=false;}
    else if(pointers.size===2){hadMultiplePointers=true;const pair=pairGesture();pinchDistance=pair.distance;pinchCenter=pair.center;}
  };
  canvas.onpointermove=event=>{
    if(stroke){if(event.pointerId===stroke.id){const samples=event.getCoalescedEvents?.()||[];let changed=false;for(const sample of samples.length?samples:[event])changed=paintSegment(sample,false)||changed;if(changed)refreshDraft();}return;}
    const previous=pointers.get(event.pointerId);if(!previous)return;const dx=event.clientX-previous[0],dy=event.clientY-previous[1];pointers.set(event.pointerId,[event.clientX,event.clientY]);dragDistance+=Math.abs(dx)+Math.abs(dy);
    if(pointers.size===2){
      const pair=pairGesture();
      if(pinchDistance>0&&pair.distance>0)zoom(pinchDistance/pair.distance);
      if(pinchCenter)pan(pair.center[0]-pinchCenter[0],pair.center[1]-pinchCenter[1]);
      pinchDistance=pair.distance;pinchCenter=pair.center;
    } else if(pointers.size===1&&pointerButton===2){pan(dx,dy);}
    else if(pointers.size===1){camera.az-=dx*.007;camera.el=Math.max(-1.56,Math.min(1.56,camera.el+dy*.007));}requestRender();
  };
  canvas.onpointerup=event=>{
    if(stroke){if(event.pointerId===stroke.id){paintSegment(event);finishStroke(false);}return;}
    if(!pointers.has(event.pointerId))return;
    if(pointers.size===1&&!hadMultiplePointers&&dragDistance<5&&pointerButton===0)pick(event.clientX,event.clientY,event.ctrlKey||event.metaKey||event.shiftKey,event.pointerType==='touch');
    pointers.delete(event.pointerId);pinchDistance=0;pinchCenter=null;
  };
  const cancelPointer=event=>{if(stroke?.id===event.pointerId)finishStroke(true);pointers.delete(event.pointerId);pinchDistance=0;pinchCenter=null;};
  canvas.onpointercancel=cancelPointer;canvas.onlostpointercapture=cancelPointer;
  canvas.onwheel=event=>{event.preventDefault();if(stroke)return;zoom(Math.exp(event.deltaY*.001));};
  canvas.onkeydown=event=>{if(stroke)return;if(event.key==='ArrowLeft')camera.az-=.12;else if(event.key==='ArrowRight')camera.az+=.12;else if(event.key==='ArrowUp')camera.el=Math.min(1.56,camera.el+.12);else if(event.key==='ArrowDown')camera.el=Math.max(-1.56,camera.el-.12);else if(event.key==='+'||event.key==='=')camera.distance=Math.max(.03,camera.distance/1.15);else if(event.key==='-')camera.distance*=1.15;else if(event.key==='Escape')resetSelection();else return;event.preventDefault();requestRender();};

  $('fit').onclick=fitCore;
  $('fitAll').onclick=()=>{fitAll();document.querySelector('.view-menu').open=false;};
  $('focus').onclick=focusSelection;
  $('zoomIn').onclick=()=>zoom(1/1.3);$('zoomOut').onclick=()=>zoom(1.3);
  $('quickGroup').onchange=()=>{if($('quickGroup').value){choose($('quickGroup').value);focusSelection();}else resetSelection();};
  $('top').onclick=()=>{finishStroke(true);camera.az=-Math.PI/2;camera.el=1.56;requestRender();};
  $('front').onclick=()=>{finishStroke(true);camera.az=-Math.PI/2;camera.el=0;requestRender();};
  function visibilityChanged(){finishStroke(true);updateColors();renderList();requestRender();}
  $('isolate').onchange=visibilityChanged;
  for(const id of Object.values(layerControls))$(id).onchange=visibilityChanged;
  for(const id of ['filter','search'])$(id).addEventListener(id==='search'?'input':'change',visibilityChanged);
  $('orbitMode').onclick=()=>setMode('orbit');$('brushMode').onclick=()=>setMode('brush');
  $('brushRadius').oninput=()=>{$('brushRadiusValue').textContent=`${$('brushRadius').value} px`;};
  $('undoStroke').onclick=undoStroke;$('clearDraft').onclick=clearDraft;$('savePoints').onclick=savePointObject;
  $('removeManual').onclick=()=>{$('removePanel').hidden=false;};$('cancelRemove').onclick=()=>{$('removePanel').hidden=true;};$('confirmRemove').onclick=removeManualObject;
  $('clearSelection').onclick=resetSelection;
  $('category').onchange=updateButtons;
  $('newCategory').onclick=openCategoryForm;$('categoryForm').onsubmit=createCategory;$('cancelCategory').onclick=closeCategoryForm;$('categoryName').oninput=updateButtons;
  $('save').onclick=()=>save('label');$('ignore').onclick=()=>save('ignore');$('reset').onclick=()=>save('reset');
  $('acceptProposal').onclick=()=>{const items=selectedGroups();if(items.length&&items.every(group=>group.suggested_label===items[0].suggested_label))save('label',items[0].suggested_label);};
  $('export').onclick=exportDataset;$('reload').onclick=reloadAnnotations;
  $('dataset').onchange=()=>{const id=$('dataset').value;if(state.draft.size){state.pendingDataset=id;$('dataset').value=state.datasetId;$('discardPanel').hidden=false;document.querySelector('aside').scrollTop=0;}else loadDataset(id);};
  $('discardAndSwitch').onclick=()=>{if(state.busy)return;const id=state.pendingDataset;state.pendingDataset=null;$('discardPanel').hidden=true;if(id){clearDraft();$('dataset').value=id;loadDataset(id);}};
  $('keepDraft').onclick=()=>{if(state.busy)return;state.pendingDataset=null;$('discardPanel').hidden=true;};
  new ResizeObserver(()=>{finishStroke(true);visibilityCache=null;requestRender();}).observe(canvas);
  Object.defineProperty(window,'atlasAnnotationHasDraft',{get:()=>state.draft.size>0});
  Object.defineProperty(window,'atlasAnnotationDebug',{get:()=>({datasetId:state.datasetId,loaded:!!state.manifest,pointCount:state.points.length,groupCount:groups().length,selected:[...state.selected],draftCount:state.draft.size,undoCount:state.undo.length,mode:state.mode,layers:Object.fromEntries(Object.entries(layerControls).map(([key,id])=>[key,$(id).checked])),revision:state.annotations?.revision,renderer:!!pointProgram,conflict:state.conflict,camera:{center:[...camera.center],distance:camera.distance},coreBounds,sceneBounds})});
  const mobileLayout=matchMedia('(max-width:760px), (pointer:coarse) and (orientation:portrait)');
  $('groupPanel').open=!mobileLayout.matches;
  mobileLayout.addEventListener('change',event=>{$('groupPanel').open=!event.matches;});
  try { initializeRenderer(); } catch(error) {pointProgram = null; $('sceneError').textContent=error.message;$('sceneError').hidden=false;}
  (async()=>{
    try {
      const result=await api('/api/datasets');
      if(!result.datasets?.length)throw new Error('No hay capturas preparadas para etiquetar.');
      $('dataset').replaceChildren(...result.datasets.map((item,index)=>new Option(captureName(item,index),item.dataset_id)));
      await loadDataset(result.datasets[0].dataset_id);
    } catch(error) {$('loading').hidden=true;$('sceneError').hidden=false;$('sceneError').textContent=error.message;failure(error);}
  })();
})();
