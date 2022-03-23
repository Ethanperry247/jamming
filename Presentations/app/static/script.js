const example = () => {
        alert("You pressed the button");
}

const button = document.getElementById('signup');
const login = document.getElementById('login');

button.addEventListener('click', (event) => {
        const username = document.getElementById('username');
        const password = document.getElementById('password');
        const data = {
                username: username.value,
                password: password.value
        }
        fetch('http://localhost:8000/signup', {
                method: 'POST', // or 'PUT'
                headers: {
                        'Content-Type': 'application/json',
                },
                body: JSON.stringify(data),
        })
                .then(response => response.json())
                .then(data => {
                        alert("Success!");
                        console.log('Success:', data);
                })
                .catch((error) => {
                        alert("Error!");
                        console.error('Error:', error);
                });
});

login.addEventListener('click', (event) => {
        const username = document.getElementById('username');
        const password = document.getElementById('password');
        const data = {
                username: username.value,
                password: password.value
        }
        fetch('http://localhost:8000/login', {
                method: 'POST', // or 'PUT'
                headers: {
                        'Content-Type': 'application/json',
                },
                body: JSON.stringify(data),
        })
                .then(response => response.json())
                .then(data => {
                        if (data.detail === "NOT_AUTHORIZED") {
                                alert("Incorrect Login!");
                        } else {
                                alert("Success!");
                                console.log('Success:', data);
                        }
                })
                .catch((error) => {
                });
});