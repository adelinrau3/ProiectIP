const username=document.getElementById("username_patient_field");
const parola=document.getElementById("password_patient_field");
const nume=document.getElementById("name_patient_field");
const prenume=document.getElementById("surname_patient_field");
const CNP=document.getElementById("cnp_patient_field");
const adresa=document.getElementById("adress_patient");
const telefon=document.getElementById("phone_number_field");
const profesie=document.getElementById("profession_field")
const job=document.getElementById("job_field");
const istoric=document.getElementById("medical_history_field");
const alergii=document.getElementById("allergies_field");
const cardio=document.getElementById("cardio_field");

const pulsMin=document.getElementById("puls_min_field");
const pulsMax=document.getElementById("puls_max_field");
const tempMin=document.getElementById("temp_min_field");
const tempMax=document.getElementById("temp_max_field");
const ecgMin=document.getElementById("ecg_min_field");
const ecgMax=document.getElementById("ecg_max_field");
const umiditateMin=document.getElementById("umiditate_min_field");
const umiditateMax=document.getElementById("umiditate_max_field");

const addButton=document.getElementById("add_button");

const database=firebase.firestore();
const pacientiCollection = database.collection("pacienti");

addButton.addEventListener("click",e => {
    e.preventDefault();
    if(!(username.value.endsWith("yahoo.com") || username.value.endsWith("gmail.com"))){
        window.alert("Username-ul trebuie sa fie o adresa de email valida! \n ex: ex@yahoo.com/ex@gmail.com");
        return;
    }
    pacientiCollection.doc(username.value).set(
        {
            nume: nume.value,
            prenume: prenume.value,
            CNP: CNP.value,
            adresa: adresa.value,
            numar_telefon: telefon.value,
            profesie: profesie.value,
            job: job.value,
            istoric_medical: istoric.value,
            alergii: alergii.value,
            cons_cardiologice: cardio.value,
            puls_minim: pulsMin.value,
            puls_maxim: pulsMax.value,
            temp_minim: tempMin.value,
            temp_maxim: tempMax.value,
            ecg_minim: ecgMin.value,
            ecg_maxim: ecgMax.value,
            umiditate_min: umiditateMin.value,
            umiditate_max: umiditateMax.value

        }).then(()=> {
            secondaryApp.auth().createUserWithEmailAndPassword(document.getElementById("username_patient_field").value, document.getElementById("password_patient_field").value);
            window.alert("Pacientul cu email-ul: "+document.getElementById("username_patient_field").value+" a fost creat.");
            document.getElementById("username_patient_field").value="";
            document.getElementById("password_patient_field").value="";
            document.getElementById("name_patient_field").value="";
            document.getElementById("surname_patient_field").value="";
            document.getElementById("cnp_patient_field").value="";
            document.getElementById("adress_patient").value="INTRODUCETI ADRESA...";
            document.getElementById("phone_number_field").value="";
            document.getElementById("profession_field").value="";
            document.getElementById("job_field").value="";
            document.getElementById("medical_history_field").value="";
            document.getElementById("allergies_field").value="";
            document.getElementById("cardio_field").value="";
            document.getElementById("puls_min_field").value="";
            document.getElementById("puls_min_field").value="";
            document.getElementById("puls_max_field").value="";
            document.getElementById("temp_min_field").value="";
            document.getElementById("temp_max_field").value="";
            document.getElementById("ecg_min_field").value="";
            document.getElementById("ecg_max_field").value="";
            document.getElementById("umiditate_min_field").value="";
            document.getElementById("umiditate_max_field").value="";
            console.log("succes!");
        }).catch(error => {
            console.error(error)
        });
});

function cancel_add(){
    document.getElementById("user_doctor_div").style.display = "block";
    document.getElementById("doctor_add").style.display = "none";
    document.getElementById("menu-span").style.display = "none";
    document.getElementById("info_pacient").style.display = "none";   
}

function add()
{
    firebase.initializeApp(config);
    var firestore = firebase.firestore();

    const docRef=firestore.doc("pacienti/");
    const username=document.querySelector("#username_patient");
    const add=document.querySelector("#add_button");

    add.addEventListener("click",function()
    {
        const text=username.value;
        docRef.set({
            username_patient: text
        }).then(function(){
            console.log("Saved");
        }).catch(function (error){
            console.log("error ", error);
        });
    })
}