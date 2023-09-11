package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_lpnFn1_9
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=153.64,67.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=183.24,12.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=28.28,66.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=61.80,161.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=87.56,249.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=197.97,149.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=272.60,25.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=285.96,94.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=388.00,106.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=423.25,56.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=434.55,170.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=509.80,212.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=543.23,120.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=618.98,111.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=485.03,41.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=329.69,190.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=359.92,244.13");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=407.27,287.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=233.93,278.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=175.28,206.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=264.91,343.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=200.23,377.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=153.19,298.81");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=259.07,204.74");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=26.11,271.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=23.79,363.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=58.02,425.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=15.46,490.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=78.37,536.61");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=106.23,466.67");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=56.78,599.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=84.53,694.75");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=19.86,667.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=145.66,631.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=163.58,492.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=160.50,743.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=149.35,812.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=88.98,795.49");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=244.41,767.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=337.16,801.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=250.57,620.08");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=123.80,368.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=256.70,682.86");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=293.29,265.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=477.89,281.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=582.07,317.45");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=658.29,319.16");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=642.01,256.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=720.93,315.85");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=676.49,377.10");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=596.97,427.85");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=653.99,500.14");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=575.83,484.76");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=608.94,553.73");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=538.06,597.99");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=513.53,680.41");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=495.24,775.74");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=562.43,780.14");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=522.00,842.56");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=646.64,755.08");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=711.42,742.85");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=756.12,693.56");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=694.37,685.12");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=623.95,828.75");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=700.35,835.93");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=792.14,792.74");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=689.52,597.81");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=707.44,530.63");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=762.59,213.93");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=695.39,223.24");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=479.24,344.34");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=459.80,115.37");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=0.06,154.27");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=335.55,329.44");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=74.57,325.48");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=270.01,505.33");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=353.40,540.22");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=430.48,496.55");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=394.87,445.43");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=489.10,427.20");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=291.18,401.46");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=314.11,595.12");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=183.94,866.69");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=49.10,850.65");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=107.64,920.37");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=263.90,873.41");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=297.65,946.81");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=356.90,991.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=323.43,1056.88");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=254.17,1079.77");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=187.79,1119.09");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=127.64,1155.89");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=63.77,1152.67");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=105.99,1074.28");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=63.58,1013.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=171.41,945.88");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=340.86,660.41");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=338.43,721.97");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=170.75,1048.94");
    }

}

