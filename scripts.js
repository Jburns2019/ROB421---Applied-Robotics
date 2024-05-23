function make_all_expandables_expandable() {
    var acc = document.getElementsByClassName("expandable");

    for (var i = 0; i < acc.length; i++) {
        acc[i].onclick = function() {
            this.classList.toggle("active");
            
            var panel = this.nextElementSibling;
            if (panel.style.maxHeight)
                panel.style.maxHeight = null
            else
                panel.style.maxHeight = panel.scrollHeight + "px";
        }
    }
}