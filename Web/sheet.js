
var sheet_count=0;
function view_sheet(){
  document.getElementById("user_doctor_div").style.display = "none";
  document.getElementById("doctor_add").style.display = "none";
  document.getElementById("menu-span-patient").style.display = "block";
  document.getElementById("info_pacient").style.display = "none";
  document.getElementById("user_patient_div").style.display = "none";
  document.getElementById("patient_details_sheet").style.display = "block";
  document.getElementById("recommandation").style.display = "none";
  document.getElementById("dinamic_table").style.display = "none";

  var patient = firebase.auth().currentUser;

  usersCollection.doc(patient.email).get()
  .then(user => {
  
    document.getElementById("username_patient_sheet").value = patient.email;
    document.getElementById("username_patient_sheet").setAttribute("readonly",true);

    document.getElementById("name_patient_sheet").value = user.data().nume;
    document.getElementById("name_patient_sheet").setAttribute("readonly",true);

    document.getElementById("surname_patient_sheet").value = user.data().prenume;
    document.getElementById("surname_patient_sheet").setAttribute("readonly",true);

    document.getElementById("cnp_patient_sheet").value = user.data().CNP;
    document.getElementById("cnp_patient_sheet").setAttribute("readonly",true);

    document.getElementById("adress_patient_sheet").value = user.data().adresa;
    document.getElementById("adress_patient_sheet").setAttribute("readonly",true);

    document.getElementById("phone_number_patient_sheet").value = user.data().numar_telefon;
    document.getElementById("phone_number_patient_sheet").setAttribute("readonly",true);
    
    document.getElementById("job_patient_sheet").value = user.data().job;
    document.getElementById("job_patient_sheet").setAttribute("readonly",true);

    document.getElementById("profession_patient_sheet").value = user.data().profesie;
    document.getElementById("profession_patient_sheet").setAttribute("readonly",true);

    document.getElementById("medical_history_patient_sheet").value = user.data().istoric_medical;
    document.getElementById("medical_history_patient_sheet").setAttribute("readonly",true);

    document.getElementById("allergies_patient_sheet").value = user.data().alergii;
    document.getElementById("allergies_patient_sheet").setAttribute("readonly",true);

    document.getElementById("cardio_patient_sheet").value = user.data().cons_cardiologice;
    document.getElementById("cardio_patient_sheet").setAttribute("readonly",true);

    
    document.getElementById("puls_min_patient_sheet").value = user.data().puls_minim;
    document.getElementById("puls_min_patient_sheet").setAttribute("readonly",true);

    document.getElementById("puls_max_patient_sheet").value = user.data().puls_maxim;
    document.getElementById("puls_max_patient_sheet").setAttribute("readonly",true);

    document.getElementById("temp_min_patient_sheet").value = user.data().temp_minim;
    document.getElementById("temp_min_patient_sheet").setAttribute("readonly",true);
    
    document.getElementById("temp_max_patient_sheet").value = user.data().temp_maxim;
    document.getElementById("temp_max_patient_sheet").setAttribute("readonly",true);

    document.getElementById("ecg_min_patient_sheet").value = user.data().ecg_minim;
    document.getElementById("ecg_min_patient_sheet").setAttribute("readonly",true);

    document.getElementById("ecg_max_patient_sheet").value = user.data().ecg_maxim;
    document.getElementById("ecg_max_patient_sheet").setAttribute("readonly",true);

    document.getElementById("umiditate_min_patient_sheet").value = user.data().umiditate_min;
    document.getElementById("umiditate_min_patient_sheet").setAttribute("readonly",true);

    document.getElementById("umiditate_max_patient_sheet").value = user.data().umiditate_max;
    document.getElementById("umiditate_max_patient_sheet").setAttribute("readonly",true);
  })
  .catch(error =>{
    console.error(error);
  });
}

function recommandation()
{
  document.getElementById("user_doctor_div").style.display = "none";
  document.getElementById("doctor_add").style.display = "none";
  document.getElementById("menu-span-patient").style.display = "block";
  document.getElementById("info_pacient").style.display = "none";
  document.getElementById("user_patient_div").style.display = "none";
  document.getElementById("patient_details_sheet").style.display = "none";
  document.getElementById("recommandation").style.display = "block";
  document.getElementById("dinamic_table").style.display = "none";

  var patient = firebase.auth().currentUser;

  usersCollection.doc(patient.email).get()
  .then(user => {
  
    document.getElementById("puls_min_patient_rec").value = user.data().puls_minim;
    document.getElementById("puls_max_patient_rec").value = user.data().puls_maxim;
    document.getElementById("temp_min_patient_rec").value = user.data().temp_minim;
    document.getElementById("temp_max_patient_rec").value = user.data().temp_maxim;
    document.getElementById("ecg_min_patient_rec").value = user.data().ecg_minim;
    document.getElementById("ecg_max_patient_rec").value = user.data().ecg_maxim;
    document.getElementById("umiditate_min_patient_rec").value = user.data().umiditate_min;
    document.getElementById("umiditate_max_patient_rec").value = user.data().umiditate_max;


  })
  .catch(error =>{
    console.error(error);
  });

  const alertsCollection = db.collection('pacienti').doc(patient.email).collection('alerte');
  document.getElementById("profession_patient_rec").value = "" ;

  alertsCollection.get()
  .then(snapshot =>{
    snapshot.forEach(patient => {
      document.getElementById("profession_patient_rec").value += patient.id + "\n";
    });
  })    
}

function view_values(){
  document.getElementById("user_doctor_div").style.display = "none";
  document.getElementById("doctor_add").style.display = "none";
  document.getElementById("menu-span-patient").style.display = "block";
  document.getElementById("info_pacient").style.display = "none";
  document.getElementById("user_patient_div").style.display = "none";
  document.getElementById("patient_details_sheet").style.display = "none";
  document.getElementById("recommandation").style.display = "none";
  document.getElementById("dinamic_table").style.display = "block";

  var table = document.getElementById("table_parameters_sheet");
  var patient = firebase.auth().currentUser;
  const parametersCollection = db.collection('pacienti').doc(patient.email).collection('parametri');

  while(sheet_count>0){
    document.getElementById("table_parameters_sheet").deleteRow(sheet_count);
    sheet_count--;

  }
  parametersCollection.get()
  .then(snapshot =>{
    snapshot.forEach(param => {
      var row = table.insertRow(sheet_count+1);
      var cell0 = row.insertCell(0);
      var cell1 = row.insertCell(1);
      var cell2 = row.insertCell(2);
      var cell3 = row.insertCell(3);

      cell0.innerHTML = param.data().puls;
      cell1.innerHTML = param.data().temperatura;
      cell2.innerHTML = param.data().umiditate;
      cell3.innerHTML = param.data().ecg;

      sheet_count++;
    });
  })
}