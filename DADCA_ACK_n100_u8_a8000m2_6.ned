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

network DADCA_ACK_n100_u8_a8000m2_6
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
        @display("i=misc/sensor;p=2771.75,7822.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3221.02,7804.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3840.35,7893.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4657.65,7956.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5108.04,7840.54");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2193.52,7571.69");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3875.08,7522.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4232.52,7678.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4548.32,7515.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5753.01,7552.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3526.00,7384.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5292.68,7471.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=409.87,7195.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=908.14,7138.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1577.58,7211.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2051.76,7056.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2662.87,7192.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3054.80,7246.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4042.22,7041.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4717.62,7207.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5596.86,7163.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1347.52,6931.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2823.45,6852.05");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3195.73,6792.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3694.35,6980.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5303.13,6908.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=545.29,6727.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=992.30,6602.56");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1828.75,6660.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2481.39,6660.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3911.43,6687.99");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4349.90,6636.50");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4797.82,6578.97");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5601.10,6674.52");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1595.18,6383.67");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2167.28,6470.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2991.88,6435.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3628.87,6446.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=672.12,6035.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1105.14,6132.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1593.54,6020.96");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1968.54,6115.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3433.80,6114.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4378.00,6063.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5100.69,6242.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5981.81,6172.97");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=165.53,5978.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2394.66,6003.15");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2854.47,5961.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3280.19,5785.09");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3926.53,5862.20");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5624.01,6008.14");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1122.82,5593.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1500.94,5637.28");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1972.60,5571.93");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4489.40,5552.24");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4946.56,5772.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=106.81,5520.31");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=558.76,5459.83");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2505.31,5289.46");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2896.61,5484.95");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3597.43,5483.21");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5487.38,5519.09");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5977.40,5333.87");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=542.61,5083.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1795.47,5209.35");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2197.81,5062.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2837.46,5130.65");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3908.64,5169.30");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4988.04,5074.75");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5728.08,5074.47");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1173.20,4952.44");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1589.43,4805.95");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3408.04,4880.25");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4538.14,5006.88");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5395.96,4868.82");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=108.93,4646.58");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=545.17,4642.53");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1334.46,4562.40");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2006.72,4553.10");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2458.64,4589.59");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2804.04,4772.03");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3952.60,4691.77");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4703.14,4647.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5076.72,4551.06");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1003.49,4322.67");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3009.71,4409.71");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3685.52,4294.37");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5451.47,4494.64");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5894.06,4479.74");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=362.13,4137.17");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1599.61,4192.35");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1952.89,4139.22");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2615.04,4255.30");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4180.66,4219.45");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4763.32,4132.82");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5442.72,4080.09");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=832.54,4003.54");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1225.28,4023.95");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2309.91,3931.93");
    }

}

