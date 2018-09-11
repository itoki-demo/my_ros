//
//  ConferenceViewController.swift
//  NavigationBot
//
//  Created by 浜田 on 2018/09/04.
//  Copyright © 2018 swiftbigg. All rights reserved.
//

import UIKit

class ConferenceViewController: UIViewController {

    var b_Info:ReceptionViewController.bookInfoJson?
    var s_Info:[sensorInfoJson]?
    
    struct sensorInfoJson: Codable{
        var atmopressure:String?
        var brightness:Float?
        var co2:Float?
        var discomfort:Float?
        var heatstrokerisk:Float?
        var temperature:Float?
        var humidity:Float?
        var time:String?
        var ultraviolet:Float?
        var volume:Float?
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        getSensorInfo()
        // Do any additional setup after loading the view.
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    @IBOutlet weak var temperatureLabel: UILabel!
    @IBOutlet weak var humidityLabel: UILabel!
    @IBOutlet weak var co2Label: UILabel!
    @IBOutlet weak var brightnessLabel: UILabel!
    @IBOutlet weak var atmopressureLabel: UILabel!
    @IBOutlet weak var discomfortLabel: UILabel!
    @IBOutlet weak var heatstrokeriskLabel: UILabel!
    @IBOutlet weak var ultravioletLabel: UILabel!
    @IBOutlet weak var volumeLabel: UILabel!
    
    
    func getSensorInfo(){
        //let ipAddress = IPAddress.shared
        let ipAddress = "http://" + IPAddress
        let phpFileName = "get_sensorValue.php"
        //var request = URLRequest(url: URL(string: "\(ipAddress.path)/\(phpFileName)")!)
        var request = URLRequest(url: URL(string: "\(ipAddress)/\(phpFileName)")!)
        request.httpMethod = "POST"
        
        let task = URLSession.shared.dataTask(with: request, completionHandler:{
            (data, response, error) in
            
            if error != nil {
                return
            }
            do{
                let decoder = JSONDecoder()
                self.s_Info = try decoder.decode([sensorInfoJson].self, from: data!)
                DispatchQueue.main.async(execute:{
                    self.temperatureLabel.text = String(self.s_Info![0].temperature!)
                    self.humidityLabel.text = String(self.s_Info![0].humidity!)
                    self.co2Label.text = String(self.s_Info![0].co2!)
                    self.brightnessLabel.text = self.s_Info![0].time!
                    
                })
            }catch{
                print(error)
            }
        })
        task.resume()
    }
    
    @IBAction func doneButtonAction(_ sender: Any) {
        
        updateStatus(status: "navigation")
        
        let storyboard: UIStoryboard = self.storyboard!
        let nextView = storyboard.instantiateViewController(withIdentifier: "Navigation") as! NavigationViewController
        nextView.b_Info = self.b_Info!
        self.present(nextView, animated: true, completion: nil)
    }
    
    func updateStatus(status:String){
        //let ipAddress = IPAddress.shared
        let ipAddress = "http://" + IPAddress
        let phpFileName = "update_status.php"
        let status = "status=\(status)"
        //var request = URLRequest(url: URL(string: "\(ipAddress.path)/\(phpFileName)?\(status)")!)
        var request = URLRequest(url: URL(string: "\(ipAddress)/\(phpFileName)?\(status)")!)
        request.httpMethod = "POST"
        let task = URLSession.shared.dataTask(with: request, completionHandler:{
            (data, response, error) in
            if error != nil {
                return
            }
            do{
            }
        })
        task.resume()
    }
    
    
    

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destinationViewController.
        // Pass the selected object to the new view controller.
    }
    */

}
