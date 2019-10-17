function updateTextInput(val) {
	          document.getElementById('textInput').value=val; 
		          }
jQuery(document).ready(function(){

	    jQuery('.ajaxform').submit( function() {

		            $.ajax({
			                url     : "./settings",
			                type    : $(this).attr('method'),
			                data    : $(this).serialize(),
			            });

			            return false;
				        });

});
function UpdateDistance(){
$.getJSON("https://api.myjson.com/bins/osxuo", function(json) {
    console.log(json); // this will show the info it in firebug console
	console.log(json.data[0].distance);
	labels = []
	distance = []
	for (var i in json.data){
		if (i%15 == 0){
			labels.push(i);
		}else{
			labels.push(" ");
		}
		distance.push(json.data[i]);
	}
	myChart.data.labels = labels;
	myChart.data.datasets[0].data=distance;
	myChart.update();

	});
}

var ctx = document.getElementById('myChart');
var myChart = new Chart(ctx, {
    type: 'radar',
    data: {
        labels: [],
        datasets: [{
            label: '# of Votes',
            data: [],
            backgroundColor: [
                'rgba(25, 207, 30, 0.2)'
            ],
            borderColor: [
                'rgba(255, 99, 132, 1)'
            ],
            borderWidth: 1
        }]
    },
    options: {

    }
});
UpdateDistance();
//setInterval(UpdateDistance, 1000);

