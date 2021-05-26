const db=firebase.firestore();
const usersCollection = db.collection('pacienti');

var user_type="none";
firebase.auth().onAuthStateChanged(function(user) {
    if (user) {
      // User is signed in.
      document.getElementById("login_div").style.display = "none";
  
      document.getElementById("loggedin-header").style.display = "block";
      document.getElementById("login-header").style.display = "none";
      document.getElementById("doctor_add").style.display = "none";
      document.getElementById("menu-span").style.display = "none"; 
      document.getElementById("menu-span-patient").style.display = "none"; 
      document.getElementById("info_pacient").style.display = "none";
      document.getElementById("patient_details_sheet").style.display = "none";  
      document.getElementById("dinamic_table").style.display = "none";
      
      var user = firebase.auth().currentUser;
      
      if(user != null){
        var email_id = user.email;
        document.getElementById("user_doctor").innerHTML = "Bine ai venit : " + email_id;
        usersCollection.get()
        .then(snapshot =>{
          snapshot.forEach(patient => {
            if(user.email == patient.id){
              document.getElementById("user_patient_div").style.display = "block";
              document.getElementById("user_doctor_div").style.display = "none";
              document.getElementById("menu-span").style.display = "none";
              document.getElementById("info_pacient").style.display = "none";
              document.getElementById("menu-span-patient").style.display = "none"; 
              document.getElementById("recommandation").style.display = "none";
              document.getElementById("dinamic_table").style.display = "none";

              user_type="patient";  
            }
          });
        })
        if(user_type != "patient"){
          document.getElementById("user_patient_div").style.display = "none";
          document.getElementById("user_doctor_div").style.display = "block";
          document.getElementById("menu-span").style.display = "none";
          document.getElementById("info_pacient").style.display = "none";   
          document.getElementById("alerts_div").style.display = "none";
          document.getElementById("menu-span-patient").style.display = "none"; 
          document.getElementById("recommandation").style.display = "none";
          document.getElementById("dinamic_table").style.display = "none";

        }
      }
  
    } else {
      // No user is signed in.
      document.getElementById("loggedin-header").style.display = "none";
      document.getElementById("login-header").style.display = "block";
      document.getElementById("login_div").style.display = "block";
      document.getElementById("user_patient_div").style.display = "none";
      document.getElementById("user_doctor_div").style.display = "none";
      document.getElementById("doctor_add").style.display = "none";
      document.getElementById("menu-span").style.display = "none";
      document.getElementById("info_pacient").style.display = "none";
      document.getElementById("alerts_div").style.display = "none"; 
      document.getElementById("patient_details_sheet").style.display = "none";   
      document.getElementById("menu-span-patient").style.display = "none";  
      document.getElementById("recommandation").style.display = "none";
      document.getElementById("dinamic_table").style.display = "none";
    }
  });
  
  function login(){
  
    var userEmail = document.getElementById("email_field").value;
    var userPass = document.getElementById("password_field").value;

    usersCollection.get()
    .then(snapshot =>{
      snapshot.forEach(patient => {
        if(userEmail == patient.id){
          user_type = "patient";
        }
      });
    })
    firebase.auth().signInWithEmailAndPassword(userEmail, userPass).catch(function(error) {
    // Handle Errors here.
    var errorCode = error.code;
    var errorMessage = error.message;

    window.alert("Error : " + errorMessage);

    // ...
    });
  
  }
  
  function add_patient(){
    if(button==true){
      button=false;
      btn_update.remove();
    }
    document.getElementById("user_doctor_div").style.display = "none";
    document.getElementById("doctor_add").style.display = "block";
    document.getElementById("menu-span").style.display = "block";
    document.getElementById("info_pacient").style.display = "none";   
    document.getElementById("alerts_div").style.display = "none";   
  }
  
  function logout(){
    user_type="none";
    firebase.auth().signOut();
  }
