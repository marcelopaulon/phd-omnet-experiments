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

network DADCA_ACK_n100_u8_a8000m2_0
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
        @display("i=misc/sensor;p=2256.22,7848.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2831.66,7803.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3350.26,7935.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4667.19,7922.89");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5093.51,7765.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5526.82,7906.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5961.07,7921.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=26.16,7561.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=442.02,7686.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=932.76,7539.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1839.20,7750.44");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3832.39,7510.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4187.00,7665.06");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4600.40,7557.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1335.56,7297.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1876.64,7372.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2710.34,7455.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3322.77,7261.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5478.40,7473.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5910.03,7368.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=513.80,7078.20");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1644.73,7031.22");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2119.03,7058.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2672.43,7017.19");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4655.56,7188.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5335.51,7060.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=30.72,6979.18");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1153.38,6953.36");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2967.54,6785.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3463.19,6862.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4006.91,6974.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=370.29,6590.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1321.81,6559.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1711.28,6550.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3732.20,6569.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4090.81,6600.32");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4654.51,6519.92");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5105.17,6724.65");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5866.84,6698.18");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=0.50,6514.06");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=754.06,6505.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2408.95,6502.89");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=409.41,6096.92");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2062.87,6051.57");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3040.32,6258.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3600.36,6152.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4124.12,6136.33");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4547.92,6135.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5167.94,6250.94");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5547.45,6190.99");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1223.81,5807.32");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1712.02,5980.00");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2487.92,5873.85");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5273.61,5835.63");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5871.74,5943.69");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=24.10,5751.22");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=531.14,5727.93");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2944.90,5597.54");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3333.08,5560.34");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3876.48,5527.34");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4220.28,5677.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=90.34,5345.08");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=715.44,5299.37");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1556.93,5312.90");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2061.77,5457.51");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2549.59,5495.91");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4712.97,5452.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5691.56,5410.50");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2325.71,5194.14");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2691.70,5090.99");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3620.66,5103.89");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4176.58,5128.45");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=448.21,4891.64");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1237.28,4879.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1833.11,4998.65");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3223.97,4974.98");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4548.55,4845.55");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5524.54,4856.39");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5961.87,5006.28");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1381.97,4553.74");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2585.43,4573.42");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3173.17,4578.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4223.69,4664.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=861.45,4443.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1962.54,4463.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3731.82,4452.57");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4493.88,4352.52");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5068.31,4519.12");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5546.30,4451.34");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=415.26,4249.61");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1189.07,4122.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1696.83,4060.36");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2412.93,4099.58");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3116.52,4212.91");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4210.99,4054.69");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4929.90,4177.48");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=188.85,3929.00");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3456.96,3805.07");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3797.79,4006.41");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4678.81,3849.60");
    }

}

