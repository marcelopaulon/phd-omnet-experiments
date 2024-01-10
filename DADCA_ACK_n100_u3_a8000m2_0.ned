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

network DADCA_ACK_n100_u3_a8000m2_0
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
        @display("i=misc/sensor;p=1013.85,7874.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1696.88,7912.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2126.88,7984.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2646.37,7999.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3389.79,7874.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4340.94,7953.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5267.35,7800.79");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=261.52,7539.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2040.63,7626.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2927.99,7615.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3669.28,7519.84");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4037.17,7746.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4868.82,7532.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5782.39,7664.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=574.14,7374.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1070.25,7490.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1528.65,7270.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2385.09,7485.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3313.73,7458.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4429.52,7421.56");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5584.14,7370.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5980.38,7309.11");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=148.02,7159.70");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2763.22,7224.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4085.76,7087.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4534.70,7050.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5001.04,7039.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=665.34,6840.77");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1138.92,6827.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1531.33,6836.13");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1898.08,6997.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2391.87,6996.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3608.66,6917.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5646.29,6956.58");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=217.31,6539.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2017.81,6614.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2840.04,6691.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3520.20,6530.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3938.43,6675.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4616.01,6614.51");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5496.30,6613.38");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5993.94,6606.35");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2317.21,6351.76");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2969.66,6296.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4329.32,6277.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5757.13,6347.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=133.50,6110.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1006.35,6246.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1767.24,6113.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3296.26,6169.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4976.66,6226.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5344.53,6251.00");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=432.11,5868.59");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1193.56,5912.84");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3857.48,6002.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4395.54,5862.91");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5720.60,5962.82");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=36.33,5540.78");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1745.69,5671.57");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2237.59,5696.29");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2813.11,5694.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3497.74,5602.64");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4852.96,5771.16");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5182.68,5638.06");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=905.19,5441.26");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1576.64,5344.94");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4048.19,5410.39");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4608.30,5454.07");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5794.82,5344.17");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=516.23,5206.50");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2105.66,5261.26");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2659.89,5171.49");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3559.77,5174.33");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5197.88,5249.66");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5529.18,5100.93");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=557.41,4814.37");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3258.19,4950.82");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3968.54,5028.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4500.50,4982.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4961.67,4941.21");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5974.43,4798.99");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=90.81,4585.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1031.14,4617.10");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1768.30,4743.64");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2195.09,4775.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2564.86,4678.49");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3403.21,4628.72");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3763.73,4649.42");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5527.81,4696.98");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1936.00,4323.95");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2853.52,4452.34");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4949.58,4515.39");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5988.48,4379.73");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=228.66,4088.07");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=628.28,4062.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=926.26,4281.21");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1477.00,4143.21");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2447.61,4271.88");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3800.30,4177.25");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4223.26,4264.39");
    }

}

