 // Your web app's Firebase configuration
  // For Firebase JS SDK v7.20.0 and later, measurementId is optional
  var firebaseConfig = {
    apiKey: "AIzaSyDP8d6KliCK6ONe2qGxb_D5FL3tSFi22-o",
    authDomain: "smartmedip.firebaseapp.com",
    projectId: "smartmedip",
    storageBucket: "smartmedip.appspot.com",
    messagingSenderId: "353817427876",
    appId: "1:353817427876:web:e87c1400616c3da237cb31",
    measurementId: "G-SCBFNFFT1E"
  };
  // Initialize Firebase
  firebase.initializeApp(firebaseConfig);
  var secondaryApp = firebase.initializeApp(firebaseConfig, "Secondary");
  firebase.analytics();