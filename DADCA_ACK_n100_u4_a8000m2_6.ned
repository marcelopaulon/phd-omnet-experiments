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

network DADCA_ACK_n100_u4_a8000m2_6
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
        @display("i=misc/sensor;p=910.01,7766.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1840.77,7878.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2238.86,7814.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2702.36,7766.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3213.83,7928.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3593.71,7757.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4410.52,7755.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4922.35,7861.27");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=260.13,7643.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1390.01,7574.19");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2971.39,7536.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4008.97,7626.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=80.49,7312.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=595.51,7351.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3394.65,7348.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3832.90,7263.78");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4446.00,7369.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4880.71,7341.70");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1036.83,7171.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1588.95,7242.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2003.21,7238.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2660.89,7010.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5224.22,7038.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5805.37,7082.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=738.71,6767.24");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1597.97,6826.27");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1955.41,6808.96");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3213.49,6954.09");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3558.13,6884.31");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1116.46,6593.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3945.82,6723.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4401.34,6557.68");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4873.27,6752.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5874.11,6603.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1599.05,6395.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1999.59,6370.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2442.73,6386.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2772.92,6509.54");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3325.86,6305.74");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3680.73,6489.02");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4896.57,6321.91");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5321.99,6347.51");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3793.80,6125.17");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4481.51,6107.75");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5841.83,6107.25");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=837.52,5943.27");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2008.72,5937.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3387.44,5924.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5057.60,5840.00");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=271.16,5732.14");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1430.84,5739.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2968.88,5758.94");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3384.96,5538.94");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3730.00,5663.65");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4156.16,5612.77");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4739.49,5618.74");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5401.92,5720.88");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=12.78,5389.33");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=579.39,5293.75");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=929.06,5524.43");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1398.94,5373.29");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2478.59,5391.16");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2838.36,5322.73");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1828.23,5226.69");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2197.57,5095.56");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3245.62,5180.85");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3650.67,5187.46");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4289.00,5202.91");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5177.53,5178.31");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5846.73,5157.26");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=956.31,4848.48");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1359.19,5018.34");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2476.59,4783.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3208.01,4790.82");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4750.90,4809.24");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=335.42,4769.84");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1716.16,4768.21");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2034.75,4539.48");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3785.74,4695.61");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5131.07,4609.58");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5619.42,4724.79");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1242.13,4410.18");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1705.07,4313.33");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2982.84,4518.11");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4011.25,4343.63");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4471.55,4491.25");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5435.72,4323.07");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5985.08,4503.96");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=296.76,4214.84");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=743.63,4192.91");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2302.99,4277.12");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2926.89,4145.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3558.80,4247.12");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4373.78,4128.39");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4878.05,4241.37");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1193.39,3923.49");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1990.97,3839.99");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2587.59,3958.54");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3382.57,3890.23");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4618.10,3877.73");
    }

}

