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

network DADCA_ACK_n100_u16_a8000m2_4
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
        @display("i=misc/sensor;p=964.77,7794.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1412.93,7940.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1929.76,7997.61");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2268.68,7792.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2896.51,7939.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3384.98,7894.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4083.33,7995.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4468.34,7819.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4923.68,7910.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5597.62,7945.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=326.64,7732.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1564.13,7562.85");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2652.92,7626.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3812.04,7671.90");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1276.42,7257.68");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2059.66,7364.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3164.03,7377.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3569.31,7291.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4334.33,7441.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4959.39,7492.45");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5419.97,7373.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5978.24,7480.11");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=340.57,7248.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=935.35,7169.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2397.18,7212.91");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4038.35,7114.16");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4686.85,7220.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5966.60,7046.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=814.39,6825.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1842.90,6927.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2307.72,6774.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2975.05,6858.46");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3455.67,6876.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4459.61,6773.74");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5046.19,6876.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=62.53,6648.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1326.92,6651.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1770.26,6538.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3803.97,6593.08");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5467.81,6688.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=464.74,6359.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1018.86,6435.10");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2240.88,6416.68");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2656.05,6452.05");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3269.78,6378.31");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4209.75,6275.39");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4648.71,6354.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5007.39,6499.42");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=917.08,6086.86");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1490.36,6223.23");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2140.55,6076.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2708.53,6059.63");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5541.16,6157.38");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=68.18,6004.85");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=523.28,5988.38");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1844.24,5856.73");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2423.05,5793.01");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3584.20,5947.66");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3960.69,5883.91");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4410.43,5856.05");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4918.41,5947.50");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5857.51,5972.28");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=430.28,5582.63");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1290.43,5586.58");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3136.91,5616.93");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3563.11,5571.58");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4866.58,5590.90");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=768.54,5489.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2059.57,5499.31");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2839.34,5341.06");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3953.36,5384.72");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4314.36,5445.70");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5428.61,5487.86");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5861.97,5353.14");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=25.78,5206.04");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1111.36,5253.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1631.15,5233.26");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2401.15,5136.97");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4598.59,5139.16");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=516.41,4980.41");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1447.30,4853.27");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2795.00,4953.61");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3234.02,5013.55");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4119.40,5018.06");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5435.51,4932.44");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5833.74,4995.38");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1974.72,4748.77");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3080.62,4605.80");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3601.91,4588.07");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4773.90,4760.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=179.87,4372.70");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=806.84,4442.51");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1387.58,4473.33");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1800.90,4425.40");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2396.44,4505.88");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4086.18,4385.53");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4509.94,4447.17");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5015.95,4404.87");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5619.25,4459.69");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5985.10,4408.36");
    }

}

