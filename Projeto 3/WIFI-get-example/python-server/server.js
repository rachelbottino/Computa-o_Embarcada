var express = require('express');
var app = express();
var bodyParser = require('body-parser');
var expressValidator = require('express-validator');
var path = require('path');
var fs = require('fs');
var busboy = require("then-busboy");
var fileUpload = require('express-fileupload');
var status;
var emb_status;
var umid;
var valor;

/*Set EJS template Engine*/
app.set('views','./views');
app.set('view engine','ejs');

app.use(express.static(path.join(__dirname, 'public')));
app.use(bodyParser.urlencoded({ extended: false })); //support x-www-form-urlencoded
app.use(bodyParser.json());
app.use(expressValidator());
app.use(fileUpload());

app.get('/', function (req, res) {
	console.log("get do /");
    if (emb_status == 'on'){
        irrig = 'ativa';
    }
    else {
        irrig = 'inativa';
    }
    console.log(irrig);
    res.render('index', {irrig:irrig, umid:umid});
 });

app.post('/', function(req, res) {

    console.log("post do /");

    if(req.method == "POST"){
        console.log("Entrou no post! \nStatus:");
        status = req.body.status;
        emb_status = req.body.emb;
        umid = req.body.umidade;
        console.log(status);
        console.log(emb_status);
        console.log(umid);
        res.redirect('/');
    } 
    else {
        res.redirect('/status');
    } 
});

app.get('/status', function(req, res) {
	console.log("get do /status");
    console.log(status);
    if (status == 'on'){
        res.send('irrigacao on');
    }
    else {
        res.send('irrigacao off');
    }
});

//start Server
var server = app.listen(3000,function(){
    console.log("Servidor rodando na porta %s",server.address().port);

});
