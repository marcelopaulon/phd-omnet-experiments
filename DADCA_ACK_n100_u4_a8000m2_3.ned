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

network DADCA_ACK_n100_u4_a8000m2_3
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
        @display("i=misc/sensor;p=290.44,7822.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1363.88,7917.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1864.77,7859.37");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4334.45,7759.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5128.75,7754.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=772.69,7637.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2471.99,7530.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3354.08,7661.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3967.81,7535.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4754.55,7659.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1668.36,7359.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5426.10,7301.99");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=210.13,7249.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=741.96,7091.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1202.25,7110.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3107.82,7048.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3692.73,7143.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4067.07,7065.61");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4698.22,7179.72");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5931.93,7059.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1593.84,6942.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2075.10,6993.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4711.98,6790.12");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5564.75,6810.16");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=685.04,6695.69");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1204.89,6733.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2332.22,6650.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2772.76,6660.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3609.25,6714.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4433.03,6534.96");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5134.30,6729.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1037.83,6292.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1396.69,6405.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1886.42,6432.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2514.98,6311.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3275.84,6495.78");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3886.97,6475.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=117.86,6045.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4779.24,6202.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5244.06,6117.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5749.54,6100.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=629.17,6008.41");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1493.30,5867.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2212.37,5819.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2805.49,5983.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3497.15,6007.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3882.64,5863.59");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4240.32,5924.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=51.08,5670.93");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=403.60,5673.77");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1048.13,5708.50");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2656.26,5573.58");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3463.24,5658.40");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4564.33,5683.73");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5497.35,5685.63");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=849.21,5326.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1521.45,5362.39");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1991.57,5278.65");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4004.94,5479.53");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4406.73,5333.32");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5071.96,5421.84");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=579.16,5082.22");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1282.30,5104.24");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2477.20,5236.67");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2958.82,5034.03");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3276.08,5231.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4803.14,5032.39");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5422.17,5117.79");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1577.25,4879.45");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2088.81,4799.00");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3736.38,4999.02");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4218.87,4921.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=355.27,4746.49");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=809.30,4715.87");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2653.66,4588.38");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3412.89,4592.06");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5039.83,4764.07");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5781.17,4712.85");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=362.08,4393.04");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1055.43,4310.49");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2323.82,4326.31");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3061.46,4412.85");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3864.64,4377.53");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4752.85,4524.02");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5477.28,4379.36");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5888.39,4294.08");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=703.12,4204.57");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1586.86,4237.85");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1093.52,3958.78");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2539.62,3977.45");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3213.61,3970.79");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4147.88,3915.71");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4624.70,3945.33");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5121.51,3826.43");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5653.37,3864.23");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=361.16,3672.68");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=759.58,3717.00");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1963.19,3596.49");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2591.55,3631.12");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3318.80,3615.98");
    }

}

