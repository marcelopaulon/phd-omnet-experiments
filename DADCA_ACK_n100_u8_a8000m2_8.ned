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

network DADCA_ACK_n100_u8_a8000m2_8
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
        @display("i=misc/sensor;p=78.18,7795.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=731.14,7975.13");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1247.94,7832.33");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1762.65,7965.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3515.55,7944.96");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5085.93,7984.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5466.43,7804.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=446.43,7518.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=858.48,7605.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2306.68,7673.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2699.27,7627.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3070.51,7574.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3859.94,7746.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4190.37,7558.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1743.26,7338.93");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4518.73,7263.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5123.50,7480.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5695.97,7362.21");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=132.86,7168.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=499.70,7140.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=874.80,7255.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2232.34,7151.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2613.66,7163.59");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3093.87,7225.29");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3597.53,7153.60");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3961.86,7169.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5106.51,7082.12");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=660.00,6815.51");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1286.01,6930.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1670.90,6896.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2381.47,6827.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4769.71,6894.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5648.53,6862.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=79.83,6540.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2018.71,6534.67");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2704.46,6667.12");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3349.60,6756.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4017.31,6623.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5909.44,6588.70");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1350.70,6497.85");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4478.93,6323.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5171.34,6428.03");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5577.11,6356.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=949.16,6182.28");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1455.28,6054.72");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1935.32,6062.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2425.64,6215.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2908.05,6206.25");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3394.52,6222.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4742.17,6038.84");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5810.44,6071.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=113.73,6017.76");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2284.75,5890.01");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3077.30,5837.78");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4135.57,5943.75");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5303.05,5820.52");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=193.92,5633.95");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=876.62,5765.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1247.27,5673.51");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2548.53,5599.96");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3617.26,5552.02");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4534.85,5675.62");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1675.58,5415.15");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2980.83,5367.25");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4436.40,5317.91");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4824.89,5444.64");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5210.15,5412.12");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5853.76,5424.19");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=24.35,5261.25");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=695.74,5225.48");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1162.55,5110.59");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2162.62,5126.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3857.88,5033.48");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5010.60,5118.89");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1492.05,4983.16");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2760.43,4971.83");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3344.31,5004.54");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5756.49,5021.41");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=16.28,4698.14");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=676.02,4605.69");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1116.74,4744.07");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2142.08,4562.59");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2550.99,4655.01");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3183.12,4671.28");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3832.02,4646.13");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4314.67,4772.61");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4872.86,4675.82");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5242.94,4765.06");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=15.55,4326.09");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=413.21,4328.06");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1065.48,4364.58");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4317.02,4379.64");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4775.15,4325.03");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5853.55,4433.81");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=732.12,4137.42");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1663.82,4269.40");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2083.33,4171.60");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2547.66,4221.02");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2961.13,4284.01");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3583.66,4279.05");
    }

}

