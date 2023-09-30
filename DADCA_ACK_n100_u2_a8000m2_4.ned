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

network DADCA_ACK_n100_u2_a8000m2_4
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
        @display("i=misc/sensor;p=234.96,7967.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=848.39,7809.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1449.64,7827.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3129.25,7992.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3690.72,7771.81");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2295.74,7646.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2780.38,7618.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3342.32,7619.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4124.69,7734.77");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4677.00,7700.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5276.25,7559.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5747.50,7518.03");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=256.93,7276.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=941.14,7387.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1366.10,7467.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1730.47,7399.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3655.16,7344.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4658.56,7344.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=816.56,7050.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1255.94,7109.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2177.20,7248.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2599.23,7078.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3096.44,7021.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4070.88,7138.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5045.89,7134.67");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5619.93,7055.84");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=102.50,6874.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=470.02,6882.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2053.55,6769.17");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3575.91,6951.66");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=751.00,6604.92");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1120.79,6676.78");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1596.81,6714.48");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3982.41,6679.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5033.24,6759.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5699.47,6516.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=386.61,6471.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1116.35,6323.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1743.89,6357.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2360.01,6505.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2782.71,6284.60");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3309.85,6374.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4449.18,6411.13");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5217.36,6401.67");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=101.26,6129.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=697.79,6241.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1443.09,6099.00");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2222.22,6090.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3754.81,6238.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4969.43,6112.33");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5830.34,6106.71");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=640.15,5813.01");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1755.42,5882.39");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2533.76,5877.56");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4570.11,5785.37");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1209.31,5633.91");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2127.45,5640.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3071.86,5538.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3652.92,5677.62");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4172.74,5753.01");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5461.82,5704.14");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=595.72,5461.71");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1624.75,5507.85");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2404.07,5317.61");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4267.01,5368.95");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5961.82,5506.44");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=480.35,5098.97");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1150.34,5084.96");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1802.62,5159.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3408.63,5174.46");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4952.07,5258.40");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5576.57,5148.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1501.46,4881.97");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2321.53,4832.00");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2722.82,5010.33");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3937.75,5014.00");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4461.37,4978.89");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5276.19,4957.23");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=735.27,4671.09");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1929.54,4667.44");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2569.96,4557.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3244.38,4763.29");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4698.71,4565.56");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5482.22,4667.88");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5978.32,4590.59");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=171.52,4491.30");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1554.50,4529.03");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3127.69,4338.52");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5157.03,4495.29");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1607.11,4106.65");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2591.48,4205.05");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3522.05,4109.79");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4151.93,4279.26");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4728.23,4162.79");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5141.91,4056.92");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5741.67,4137.80");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=164.58,3943.19");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1091.36,4033.59");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2288.34,4023.33");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2947.56,3979.52");
    }

}

