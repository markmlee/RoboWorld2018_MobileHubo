package com.example.rainbow.test3;

import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.Scanner;

public class MainActivity extends AppCompatActivity {

    private static Socket s;
    private static PrintWriter printWriter;
    private static BufferedReader bufferedReader;

    Button b1, b2, b3, b4, b5;

    String message = " ";
    private static String ip = "192.168.1.17";

    //recieved message
    String data;
    int ready_flag = 1;




    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        b1 = (Button) findViewById(R.id.button);
        b2 = (Button) findViewById(R.id.button2);
        b3 = (Button) findViewById(R.id.button3);
        b4 = (Button) findViewById(R.id.button4);
        b5 = (Button) findViewById(R.id.button5);


        Button.OnClickListener myClick = new Button.OnClickListener(){

            public void onClick(View v){

                myTask mt = new myTask();
                readBuffer rb = new readBuffer();
                ready_flag =0;


                switch(v.getId()){
                    case R.id.button:
                        Toast.makeText(getApplicationContext(),"Drink1",Toast.LENGTH_LONG).show();
                        message = "1";
                        //button_enable(false);
                        rb.execute();
                        //Toast.makeText(getApplicationContext(),"ready",Toast.LENGTH_LONG).show();
                        //button_enable(true);
                        break;
                    case R.id.button2:
                        Toast.makeText(getApplicationContext(),"Drink2",Toast.LENGTH_LONG).show();
                        message = "2";
                        rb.execute();
                        //mt.execute();
                        break;
                    case R.id.button3:
                        Toast.makeText(getApplicationContext(),"Drink3",Toast.LENGTH_LONG).show();
                        message = "3";
                        rb.execute();
                        //mt.execute();
                        break;
                    case R.id.button4:
                        Toast.makeText(getApplicationContext(),"RESET",Toast.LENGTH_LONG).show();
                        message = "4";
                        //mt.execute();
                        rb.execute();
                        break;
                    case R.id.button5:
                        Toast.makeText(getApplicationContext(),"TURN OFF",Toast.LENGTH_LONG).show();
                        message = "5";
                        //rb.execute();
                        mt.execute();
                        break;

                }
                //button_enable(true);


            }
        };

        findViewById(R.id.button).setOnClickListener(myClick);
        findViewById(R.id.button2).setOnClickListener(myClick);
        findViewById(R.id.button3).setOnClickListener(myClick);
        findViewById(R.id.button4).setOnClickListener(myClick);
        findViewById(R.id.button5).setOnClickListener(myClick);

    }




    class myTask extends AsyncTask<Void, Void, Void>
    {
        @Override
        protected Void doInBackground(Void... params)
        {
            Log.i(this.getClass().getName(),"Trying to Connect to server...");

            try{
                s = new Socket(ip,5000); //connect to the socket at port 5000

                printWriter = new PrintWriter(s.getOutputStream());  //set the output stream
                printWriter.write(message); //send the message through the socket
                printWriter.flush();
                printWriter.close();

                s.close();

            }catch (IOException e){
                e.printStackTrace();
            }

            return null;
        }
    }



    class readBuffer extends AsyncTask<Void, Void, Void>
    {
        @Override
        protected Void doInBackground(Void... params)
        {
            Log.i(this.getClass().getName(),"Trying to Connect to server...");

            try{

                button_enable(false);
                s = new Socket(ip,5000); //connect to the socket at port 5000

                printWriter = new PrintWriter(s.getOutputStream());  //set the output stream
                printWriter.write(message); //send the message through the socket
                printWriter.flush();


                Log.i(this.getClass().getName(),"data="+data + "ready_flag = "+ ready_flag);
                bufferedReader = new BufferedReader(new InputStreamReader(s.getInputStream()));

                String data;
                data = bufferedReader.readLine();
                Log.i(this.getClass().getName(),data);


                //while(true) if(data.equals("ready")) break;

                if(data.equals("ready ")) {
                    ready_flag = 1;
                    //button_enable(true);
                }

                button_enable(true);
                Log.i(this.getClass().getName(),"data="+data + "ready_flag = "+ ready_flag);

                bufferedReader.close();
                printWriter.close();
                s.close();

            }catch (IOException e)
            {
                e.printStackTrace();
            }

            return null;
        }

    }


    void button_enable(boolean b_enable)
    {
        //b1.setEnabled(b_enable);
        //b2.setEnabled(b_enable);
        //b3.setEnabled(b_enable);
        //b4.setEnabled(b_enable);
        //b5.setEnabled(b_enable);
        b1.setClickable(b_enable);
        b2.setClickable(b_enable);
        b3.setClickable(b_enable);
        b4.setClickable(b_enable);
        b5.setClickable(b_enable);
    }

}



