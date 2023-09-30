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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=386.69,7831.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=915.56,7951.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2543.92,7971.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2923.01,7933.12");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4149.17,7991.09");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4890.90,7978.29");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5243.87,7929.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1401.93,7697.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1903.56,7622.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3482.07,7725.62");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5613.49,7620.26");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=316.10,7435.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=904.07,7469.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2583.15,7424.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3121.54,7335.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3847.15,7314.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4252.16,7389.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4674.80,7433.01");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5283.71,7421.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=759.41,7091.36");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1290.22,7147.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1719.90,7236.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2193.36,7064.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3491.74,7244.20");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5887.28,7011.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=401.22,6905.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1261.57,6771.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2717.06,6997.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3610.92,6837.38");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4215.10,6930.69");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5306.44,6851.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=822.02,6723.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1798.51,6692.00");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4154.51,6572.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4896.72,6642.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3086.44,6468.23");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4495.62,6438.17");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5889.99,6507.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=580.30,6254.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1443.80,6118.72");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2152.03,6057.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2700.33,6171.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4798.03,6196.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5369.88,6266.41");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1100.13,6014.44");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1628.99,5820.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3395.59,5933.16");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4094.29,5933.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5672.05,5887.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=207.33,5751.59");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=720.75,5742.24");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1304.40,5623.74");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2096.31,5662.21");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2968.38,5559.71");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3777.26,5566.36");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4436.87,5729.46");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4959.20,5754.90");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=508.92,5383.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2425.89,5495.66");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3447.20,5341.79");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4610.66,5380.19");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5376.76,5413.27");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5840.55,5289.16");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=959.62,5220.10");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1645.73,5193.03");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2237.25,5100.87");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2709.27,5086.41");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=354.56,4976.31");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1910.44,4830.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3326.07,4890.50");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3776.10,4824.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4339.40,4969.12");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4880.89,4961.24");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=809.48,4659.77");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1333.69,4742.52");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2482.22,4740.26");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2996.78,4632.94");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4824.05,4569.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5242.79,4735.11");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5603.57,4776.15");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1978.53,4346.38");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2577.28,4345.87");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3469.39,4452.00");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3979.71,4308.40");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4497.42,4392.82");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=280.23,4246.28");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1237.71,4244.96");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1598.41,4165.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5047.30,4192.78");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5442.33,4225.47");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5828.02,4099.68");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=946.67,3891.28");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1920.69,3985.20");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2328.15,3889.96");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2713.13,3813.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3259.62,3990.86");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4179.31,3858.81");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4798.78,3872.82");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1292.92,3579.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3205.31,3625.70");
    }

}

