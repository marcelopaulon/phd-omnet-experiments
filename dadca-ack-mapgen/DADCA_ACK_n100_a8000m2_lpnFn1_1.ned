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

network DADCA_ACK_n100_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=164.10,121.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=178.04,61.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=207.07,167.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=70.24,3.03");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=31.37,99.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=8.89,1.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=172.73,1.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=234.96,12.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=62.89,194.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=265.62,148.01");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=201.67,248.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=298.83,216.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=305.57,297.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=329.66,362.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=239.45,342.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=163.95,393.94");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=389.54,353.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=351.69,431.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=418.44,291.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=246.91,460.16");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=429.44,451.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=515.61,470.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=606.26,433.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=677.05,420.50");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=742.00,385.66");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=786.91,303.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=754.08,248.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=845.37,394.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=899.31,319.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=974.19,261.65");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=1038.55,298.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=979.50,162.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=903.13,131.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=996.04,64.83");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=1051.93,4.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=1069.64,62.90");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=1128.88,38.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=1198.21,71.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1270.68,91.63");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1323.18,174.81");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=1328.29,269.36");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1229.94,219.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1187.88,171.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1123.54,130.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1100.29,192.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=903.46,236.03");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=956.58,342.18");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=850.24,284.52");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=914.98,430.26");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=547.20,391.72");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=461.27,521.48");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=404.38,596.52");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=310.67,573.75");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=288.63,649.72");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=231.35,625.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=164.69,625.17");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=194.46,531.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=286.32,757.53");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=346.71,722.34");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=387.23,768.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=360.51,836.63");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=198.94,771.92");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=157.80,842.72");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=82.72,895.11");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=33.20,962.47");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=167.11,936.45");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=213.02,992.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=134.40,1012.86");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=98.59,962.72");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=72.63,832.24");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=19.20,903.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=121.55,778.26");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=51.11,715.94");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=102.90,649.70");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=57.22,600.33");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=119.84,524.65");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=79.54,436.99");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=163.75,699.09");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=177.56,463.38");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=322.15,501.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=290.12,416.12");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=530.27,535.34");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=557.92,595.19");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=508.09,674.57");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=374.62,651.56");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=423.95,690.73");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=484.32,593.50");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=472.39,405.37");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=586.28,682.42");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=524.79,782.40");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=627.11,776.70");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=671.85,853.04");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=477.65,732.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=337.63,945.08");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=319.76,1038.34");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=230.95,925.71");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=27.78,1031.09");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=274.46,861.13");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=416.78,971.00");
    }

}

