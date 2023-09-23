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

network DADCA_ACK_n100_u16_a8000m2_9
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
        @display("i=misc/sensor;p=1175.03,7852.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2156.30,7848.36");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2520.83,7836.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3100.21,7981.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3945.13,7879.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4980.85,7860.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2789.40,7573.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3604.47,7667.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5785.08,7515.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1781.68,7454.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3575.13,7302.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4208.13,7439.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5229.46,7471.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1270.53,7196.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1709.02,7013.38");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2365.38,7217.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3017.11,7115.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3872.39,7019.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4836.25,7086.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5568.18,7059.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1995.70,6796.66");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3367.85,6970.97");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=581.65,6536.35");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1223.25,6749.82");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2317.16,6601.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2793.53,6752.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3613.55,6641.98");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3983.34,6655.22");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4453.25,6698.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5147.45,6676.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5590.59,6615.70");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5964.67,6731.21");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1021.83,6404.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1601.46,6273.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2818.86,6300.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3226.75,6385.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5022.64,6302.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5919.84,6337.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=99.24,6143.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2242.94,6123.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3146.46,6030.03");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3508.35,6027.50");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3906.43,6236.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4438.99,6246.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=730.53,5964.33");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1373.29,5958.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4714.42,6003.59");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5568.17,5985.75");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5992.65,5898.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1896.52,5649.24");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2816.79,5688.79");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3144.45,5549.30");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4158.75,5677.44");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5266.34,5611.23");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5833.13,5576.98");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=255.15,5393.70");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=641.93,5281.03");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3651.34,5381.47");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4717.58,5421.84");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1024.67,5180.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1623.79,5191.83");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1987.99,5255.12");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2368.62,5160.31");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4188.13,5048.61");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5210.24,5033.79");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5588.12,5263.45");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=411.09,4963.17");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1286.78,4908.94");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2031.81,4853.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2788.45,4998.25");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3397.52,4885.08");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4604.04,4972.93");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=901.53,4680.05");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2394.96,4683.89");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4487.12,4555.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5088.13,4677.73");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5512.05,4585.79");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=334.84,4393.93");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1768.62,4342.64");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2889.09,4434.28");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3346.26,4456.07");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3808.66,4502.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4146.72,4390.66");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1097.52,4281.11");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1525.60,4090.44");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2129.27,4148.97");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2958.39,4088.23");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3468.96,4120.58");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4317.02,4060.13");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4833.54,4070.14");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5389.33,4072.22");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=29.05,3987.31");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=771.19,3924.40");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1190.12,3837.45");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2662.65,3863.21");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5766.10,4006.52");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=90.12,3620.67");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=466.65,3706.34");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1876.65,3653.70");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3053.40,3604.93");
    }

}

