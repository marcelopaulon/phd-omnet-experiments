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
        @display("i=misc/sensor;p=100.82,7932.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1445.43,7938.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2144.75,7964.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2751.68,7877.18");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3843.35,7931.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5277.57,7914.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5696.09,7841.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=889.21,7729.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1314.71,7608.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3290.14,7684.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3729.17,7522.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4142.27,7736.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=48.56,7355.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=632.11,7415.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1687.24,7379.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2029.30,7478.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2712.44,7404.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3075.73,7292.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4595.04,7478.87");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5273.71,7444.49");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5826.28,7446.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1196.58,7045.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3596.03,7188.71");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4007.75,7044.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4731.33,7019.43");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5139.44,7069.47");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5544.41,7221.41");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1713.91,6896.45");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2533.43,6898.89");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3094.47,6892.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=383.40,6524.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1184.62,6547.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2200.43,6748.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2901.18,6579.55");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3382.53,6691.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3783.11,6704.35");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4167.66,6652.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4951.78,6619.85");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5427.96,6749.19");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=277.06,6127.15");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1626.37,6215.92");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2152.55,6174.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2508.02,6190.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3633.31,6066.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4097.98,6076.29");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4498.38,6130.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5230.10,6186.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5675.39,6249.56");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=646.57,5993.32");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1040.05,6010.50");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1906.79,5799.34");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2425.67,5846.23");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3005.33,5909.90");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4860.79,6005.20");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5960.02,5911.72");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=540.04,5566.96");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1373.49,5652.43");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2707.66,5539.81");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3153.01,5534.16");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3911.22,5730.47");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5014.48,5668.70");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5553.99,5622.77");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1034.50,5411.03");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1693.34,5508.47");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2389.03,5311.88");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3588.50,5497.79");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4364.94,5356.63");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4978.39,5278.07");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=319.78,5164.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=837.43,5078.52");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1297.62,5113.95");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5307.96,5125.55");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5796.11,5261.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=9.12,4959.03");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1654.97,4878.85");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2058.71,5021.76");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2809.63,4894.49");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3467.99,4867.64");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4050.93,4981.38");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4675.11,4994.60");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5691.03,4840.25");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=522.70,4554.03");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1058.62,4767.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1444.92,4539.74");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1997.09,4573.98");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2355.35,4770.01");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3021.86,4594.40");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3776.83,4699.79");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4566.80,4583.95");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5104.78,4604.25");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1079.96,4360.31");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2546.23,4405.95");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5433.37,4393.45");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=260.78,4187.10");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1376.56,4172.21");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1930.38,4221.77");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3664.43,4268.77");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3982.60,4066.31");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4365.20,4052.69");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5102.37,4174.56");
    }

}

