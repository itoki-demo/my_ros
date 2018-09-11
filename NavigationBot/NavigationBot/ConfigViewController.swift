//
//  ConfigViewController.swift
//  NavigationBot
//
//  Created by 浜田 on 2018/09/05.
//  Copyright © 2018 swiftbigg. All rights reserved.
//

import UIKit

var IPAddress = ""

/*
class IPAddress {//グローバル変数としてを定義
    private init() {
    }
    static let shared = IPAddress()
    var path = "http://192.168.20.42"
}
 */



class ConfigViewController: UIViewController {

    
    @IBOutlet weak var nowIPLabel: UILabel!
    @IBOutlet weak var IPTextField: UITextField!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        nowIPLabel.text = readIPAddress()
        // Do any additional setup after loading the view.
    }
    /*
    func readIPAddress() -> String{
        let filename:String = "IPAddress"
        let csvBundle = Bundle.main.path(forResource: filename, ofType: "txt")
        
        do {
            //csvBundleのパスを読み込み、UTF8に文字コード変換して、NSStringに格納
            let tsvData = try String(contentsOfFile: csvBundle!,encoding: String.Encoding.utf8)
            print(tsvData)
            return tsvData
        }
        catch {
            print("エラー")
            return ""
        }
    }
 */
    func readIPAddress() -> String{
        let file_name = "IPAddress.txt"
        
        if let dir = FileManager.default.urls( for: .documentDirectory, in: .userDomainMask ).first {
            
            let path_file_name = dir.appendingPathComponent( file_name )
            
            do {
                
                let text = try String( contentsOf: path_file_name, encoding: String.Encoding.utf8 )
                return text
            } catch {
                
            }
        }
        return ""
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    func writeIPAdrress(str:String){
        let file_name = "IPAddress.txt"
        let text = IPTextField.text //保存する内容
        print("textField = " +  text!)
        
        if let dir = FileManager.default.urls( for: .documentDirectory, in: .userDomainMask ).first {
            
            let path_file_name = dir.appendingPathComponent( file_name )
            
            do {
                
                try text!.write( to: path_file_name, atomically: false, encoding: String.Encoding.utf8 )
                
            } catch {
                //エラー処理
            }
        }
        
    }
    
    
    @IBAction func okButtonAction(_ sender: Any) {
        if(IPTextField.text != ""){
            writeIPAdrress(str: IPTextField.text!)
            print("written!")
        }
        goReception()
    }
    
    @IBAction func cancelButtonAction(_ sender: Any) {
        goReception()
    }
    
    func goReception(){
        let storyboard: UIStoryboard = self.storyboard!
        let nextView = storyboard.instantiateViewController(withIdentifier: "Reception") as! ReceptionViewController
        self.present(nextView, animated: true, completion: nil)
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
