const fileEditor = document.getElementById('editor');
const fileSelector = document.getElementById('file-select');
const statusTxt = document.getElementById('status');

// Editor auto-resize
fileEditor.addEventListener(
    'input',
    function() {
        this.style.height = 'auto';
        this.style.height = (this.scrollHeight) + 'px';
    }
);

// Display action status
function showStatus(msg) {
    statusTxt.innerText = msg;
    setTimeout(() => statusTxt.innerText = '', 5000);
}

// Initial file load
async function loadFiles() {
    const fileList = await fetch('/yaml/api/fileList');
    const files = await fileList.json();

    fileSelector.innerHTML = '<option value="">--- Select a file ---</option>';
    files.forEach(
        fileName => {
            const selectOption = document.createElement('option');
            selectOption.value = fileName;
            selectOption.innerText = fileName;
            fileSelector.appendChild(selectOption)
        }
    );
}

// File creation
async function createFile() {
    const illegalCharsRegex = /^(?!\.)[^\x00-\x1f\\?*:"<>|/]+(?<!\s)$/;
    const extCheckRegex = /\.[0-9a-z]+$/i;

    let fileName = prompt("Enter new file name (e.g. paths or paths.yaml)");
    console.log(`Received fileName: ${fileName}`);

    if (!fileName || !(illegalCharsRegex.test(fileName))) {
        console.log("No fileName or illegal fileName entered: Canceling creation")
        return;
    }
    if (!(extCheckRegex.test(fileName))) { fileName = fileName + '.yaml' }
    console.log(`Creating file with name: ${fileName}`)

    const selectorOption = document.createElement('option');
    selectorOption.value = fileName;
    selectorOption.innerText = fileName;

    fileSelector.appendChild(selectorOption);
    fileSelector.value = fileName;

    fileEditor.value = "---\n# Type your information here";
    fileEditor.dispatchEvent(new Event('input'));
    saveCurrentFile(true);
}

// Open file data for existing file
async function loadFile() {
    const fileName = fileSelector.value;
    if (!fileName) {
        fileEditor.value = '';
        return;
    }
    const fileContent = await fetch('/yaml/api/load?file=' + encodeURIComponent(fileName));

    fileEditor.value = await fileContent.text();
    fileEditor.dispatchEvent(new Event('input'));

    console.log(`Loading file with name: ${fileName}`);
    showStatus(`Loaded File: ${fileName}`);
}

// Save file to C-hub storage
async function saveCurrentFile(create) {
    const fileName = fileSelector.value;
    if (!fileName) { return alert("No file selected, please select or create one first!!"); }

    await fetch(
        '/yaml/api/save?file=' + encodeURIComponent(fileName),
        {
            method: 'POST',
            body: fileEditor.value
        }
    );
    console.log(create ? `File ${fileName} created on robot storage` : `Data from File ${fileName} transmitted and saved to robot`)
    showStatus(create ? 'Successfully Created File!' : 'Saved and Sent to Robot!')
}

async function deleteCurrentFile() {
    const fileName = fileSelector.value
    if (!fileName) { return alert("Cannot delete a file that isn't selected, please select or create a file first!"); }
    if (!(confirm(`Are you sure you want to delete file: ${fileName}`))) { return; }

    await fetch('/yaml/api/delete?file=' + encodeURIComponent(fileName))
    await loadFiles();
    fileEditor.value = ""

    console.log(`Deleted file: ${fileName}`)
    showStatus('Successfully deleted file!')
}

window.onload = loadFiles;