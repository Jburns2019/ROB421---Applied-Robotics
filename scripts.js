function toggle_visibility(id_name, button) {
    document.getElementById(id_name).classList.toggle('hidden')
    button.classList.toggle('plus')
    button.classList.toggle('minus')
}