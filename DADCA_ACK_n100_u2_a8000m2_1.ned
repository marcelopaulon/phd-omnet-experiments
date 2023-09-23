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
        @display("i=misc/sensor;p=125.57,7925.17");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=767.43,7770.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2612.19,7767.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3123.05,7991.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4561.58,7877.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5512.73,7913.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1492.95,7729.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2242.56,7617.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3473.84,7736.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3988.66,7741.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5231.27,7671.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5796.55,7626.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=154.55,7273.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1224.52,7435.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1829.48,7469.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2896.27,7446.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3458.45,7315.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4122.62,7409.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4552.82,7475.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4909.40,7448.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=654.62,7225.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1089.33,7086.75");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1792.81,7100.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2271.13,7171.33");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2660.74,7111.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4751.42,7020.15");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5408.52,7117.90");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=192.19,6880.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1472.56,6955.25");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2853.95,6793.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3471.38,6956.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4059.70,6937.77");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5739.35,6816.04");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=853.32,6688.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1828.54,6542.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2154.82,6696.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3474.56,6555.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3908.73,6557.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4796.45,6672.92");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5243.72,6563.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=26.81,6495.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1318.42,6419.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2734.45,6461.84");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3035.27,6281.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4364.49,6452.59");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5686.68,6429.95");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=471.70,6078.57");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=854.11,6191.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1680.39,6176.08");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2427.42,6201.77");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3419.48,6203.15");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3906.08,6131.43");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5090.58,6055.83");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5576.32,6077.38");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1292.90,5897.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1707.53,5784.50");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2105.22,5929.54");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2721.43,5808.35");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3126.41,5888.09");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3626.92,5778.19");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4193.79,5897.80");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4655.85,5931.42");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=66.40,5568.39");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=956.63,5722.20");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4157.52,5547.32");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5034.21,5565.80");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5858.31,5626.75");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=584.22,5401.87");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1851.20,5425.23");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2323.73,5383.08");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3039.95,5380.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3499.47,5423.73");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4487.84,5405.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=843.67,5155.99");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1417.18,5230.09");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1720.41,5047.83");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4094.76,5080.77");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4885.94,5162.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5286.64,5104.31");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5609.04,5269.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=362.59,4945.84");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2746.35,4846.47");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3648.93,4980.91");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4568.28,4940.44");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5795.67,4825.77");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1321.10,4668.51");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2116.76,4710.42");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2505.55,4574.42");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3335.05,4766.98");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4242.26,4646.45");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5126.76,4588.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=76.57,4320.13");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=757.34,4481.94");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1784.09,4435.79");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3020.12,4496.09");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3687.91,4420.55");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5492.34,4505.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1042.31,4171.27");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1390.33,4225.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2336.46,4100.32");
    }

}

