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

network DADCA_ACK_n100_a8000m2_1
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
        @display("i=misc/sensor;p=27.21,7893.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=580.39,7771.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1119.84,7816.02");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2118.37,7921.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5003.27,7818.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5732.98,7843.80");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=933.22,7493.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2039.12,7484.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=347.71,7462.27");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1626.92,7312.91");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2631.33,7460.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4680.58,7403.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5208.41,7431.27");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=32.70,7126.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1199.95,6995.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2113.67,7092.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2673.11,7032.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5001.18,7084.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5639.98,7186.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=85.82,6728.31");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=567.00,6798.56");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1924.49,6725.54");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5252.64,6732.27");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=998.94,6662.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1525.56,6567.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2225.08,6495.38");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2793.73,6669.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4808.47,6697.68");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5796.34,6593.87");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=74.37,6288.88");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=502.33,6196.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1161.08,6279.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1844.75,6326.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5931.91,6239.78");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1665.86,5969.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2682.02,5983.83");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3362.66,6070.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4066.74,6099.46");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4828.27,6184.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5293.49,6164.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5626.14,5979.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=278.76,5860.11");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=851.81,5870.30");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1299.97,5897.97");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2306.25,5715.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3007.30,5671.83");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4835.97,5755.67");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=278.83,5479.08");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1331.54,5469.30");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1783.93,5427.34");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3546.06,5650.81");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3927.76,5420.63");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4321.01,5524.95");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5347.22,5572.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=643.09,5372.42");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2237.99,5283.67");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3164.82,5326.39");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5710.36,5402.49");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=0.80,5070.81");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=970.31,5040.45");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1869.48,4966.47");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2799.19,5150.37");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3230.02,4947.19");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4194.23,5041.67");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4870.68,5127.02");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5788.93,4927.07");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=470.82,4704.06");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1500.83,4883.11");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2549.17,4768.53");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3590.66,4775.46");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4580.14,4854.67");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5222.61,4840.21");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=107.23,4411.98");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1056.24,4503.70");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1647.27,4493.88");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2118.64,4609.03");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3000.15,4425.52");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3960.61,4591.19");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4346.83,4528.06");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5853.34,4451.82");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=795.89,4215.71");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2458.80,4253.99");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3315.81,4238.52");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5105.69,4369.06");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1592.34,4036.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2113.98,4039.09");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2800.56,4075.20");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4022.89,3945.07");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4617.55,4011.48");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5068.86,3875.75");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5.29,3688.48");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=394.14,3814.25");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=774.19,3747.19");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1252.68,3812.54");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2513.69,3826.00");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2938.03,3703.51");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3596.70,3787.03");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4543.43,3619.14");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5520.15,3791.94");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5980.79,3751.23");
    }

}

