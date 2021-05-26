var cell_count=0;
function show_alert_page(){
  if(button==true){
    button=false;
    btn_update.remove();
  }
  document.getElementById("alerts_div").style.display = "block";
  document.getElementById("user_doctor_div").style.display = "none";
  document.getElementById("doctor_add").style.display = "none";
  document.getElementById("menu-span").style.display = "block";
  document.getElementById("info_pacient").style.display = "none";
  document.getElementById("details_pacient").style.display = "none";
  document.getElementById("table_pacient").style.display = "none";
  document.getElementById("patient_table").style.display = "none";
  document.getElementById("alert_details_patient").style.display = "none";

  usersCollection.get()
  .then(snapshot =>{
    var catOptions = "<option disabled selected value> Selectati un pacient </option>";
    snapshot.forEach(patient => {
      catOptions += "<option>" + patient.id + "</option>";
    });
    document.getElementById("patient_select").innerHTML = catOptions;
  })    
}

function show_patient_details(){
  document.getElementById("patient_table").style.display = "block";
  document.getElementById("alert_details_patient").style.display = "block";

  document.getElementById("name_patient_alerts").setAttribute("readonly",true);
  document.getElementById("surname_patient_alerts").setAttribute("readonly",true);
  var selected_patient = document.getElementById("patient_select").value;

  usersCollection.get()
  .then(snapshot =>{
    var table = document.getElementById("table_parameters_patient");
    snapshot.forEach(patient => {
      if(patient.id == selected_patient){
        document.getElementById("name_patient_alerts").innerHTML = patient.data().nume;
        document.getElementById("surname_patient_alerts").innerHTML = patient.data().prenume;

        const parametersCollection = db.collection('pacienti').doc(patient.id).collection('parametri');
        const alertsCollection = db.collection('pacienti').doc(patient.id).collection('alerte');

        const add_alert_btn=document.getElementById("add_alert_btn");
        add_alert_btn.addEventListener("click",e => {
          alertsCollection.doc(document.getElementById("add_alert_text").value).set({});
          window.alert("Alerta a fost adaugata cu succes pentru: "+patient.id);
          document.getElementById("add_alert_text").value="";
        });

        while(cell_count>0){
          document.getElementById("table_parameters_patient").deleteRow(cell_count);
          cell_count--;
        }

        parametersCollection.get()
        .then(snapshot =>{
          snapshot.forEach(param => {
            var row = table.insertRow(cell_count+1);
            var cell0 = row.insertCell(0);
            var cell1 = row.insertCell(1);
            var cell2 = row.insertCell(2);
            var cell3 = row.insertCell(3);

            cell0.innerHTML = param.data().puls;
            cell1.innerHTML = param.data().temperatura;
            cell2.innerHTML = param.data().umiditate;
            cell3.innerHTML = param.data().ecg;

            cell_count++;
          });
        })
      }
    });
  })
}