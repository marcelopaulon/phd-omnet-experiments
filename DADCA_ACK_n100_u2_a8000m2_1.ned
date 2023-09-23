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

network DADCA_ACK_n100_u2_a8000m2_1
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
        @display("i=misc/sensor;p=259.98,7769.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=744.92,7919.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1193.23,7851.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1717.50,7920.78");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2710.14,7835.16");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4219.82,7948.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4639.77,7959.68");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5195.41,7817.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5655.78,7759.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1791.01,7524.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3060.68,7681.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3519.73,7636.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4088.31,7565.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4840.16,7634.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=450.70,7337.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1267.39,7363.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2251.63,7470.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4365.07,7337.04");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5244.40,7419.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=73.56,7146.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=945.80,7158.51");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1564.33,7028.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3324.05,7052.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3622.53,7241.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4106.03,7023.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4674.01,7161.68");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5495.29,7104.53");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5861.54,7159.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1904.79,6937.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2922.46,6865.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3590.74,6785.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4532.81,6841.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=166.88,6701.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=534.06,6710.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1430.29,6680.06");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2372.96,6697.52");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5868.67,6554.77");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=316.01,6345.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1119.34,6376.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1667.58,6365.68");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3203.79,6386.57");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4184.49,6446.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4640.65,6472.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5192.60,6507.95");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5605.72,6304.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=923.93,6060.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2201.89,6212.28");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3790.01,6177.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2736.70,5910.48");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4087.83,5899.30");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4571.70,5956.97");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5062.64,6011.09");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5778.64,5779.44");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=297.57,5750.58");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1207.56,5690.28");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2256.59,5718.27");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2972.08,5599.74");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3409.70,5746.57");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3766.73,5744.76");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5339.06,5676.62");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=771.50,5483.76");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1657.95,5431.18");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2390.41,5298.46");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3615.49,5421.39");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4060.85,5281.29");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4422.01,5383.03");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5041.50,5433.98");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5390.72,5298.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5885.38,5367.65");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1208.35,5196.14");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2849.92,5269.89");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3223.71,5122.91");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=625.96,4955.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1935.77,4881.88");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2539.16,4936.85");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3805.68,4816.63");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4397.55,4923.19");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4861.13,5026.68");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=420.83,4554.39");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=807.63,4541.55");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2947.56,4669.97");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3492.40,4565.10");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5268.48,4774.41");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1570.18,4502.31");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2369.60,4442.98");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3895.66,4427.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4599.33,4378.87");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5619.88,4460.36");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=121.76,4064.55");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=697.22,4157.63");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1313.79,4238.27");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2829.86,4257.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3346.41,4055.62");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4242.68,4266.37");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5225.44,4079.92");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1110.73,3877.48");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1539.41,3809.60");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1863.24,4028.72");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2457.43,4027.64");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3020.53,3890.29");
    }

}

