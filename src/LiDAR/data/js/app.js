function UpdateDistance(){
$.getJSON("data", function(json) {
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
	scale: {
		ticks: {
			
		        beginAtZero: true
		}
	}
    }
});
UpdateDistance();
setInterval(UpdateDistance, 1000);

