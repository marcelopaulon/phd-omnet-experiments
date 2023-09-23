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

network DADCA_ACK_n100_u4_a8000m2_2
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
        @display("i=misc/sensor;p=742.67,7994.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2954.14,7919.62");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=247.73,7641.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=739.28,7640.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1185.39,7642.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1767.27,7694.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2226.27,7703.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3204.85,7578.01");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3657.26,7672.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4013.90,7719.08");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4824.67,7694.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5429.29,7664.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=581.89,7293.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1277.22,7268.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2234.37,7338.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2810.48,7355.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3937.00,7278.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4352.27,7351.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5151.51,7411.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5784.71,7429.92");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3184.43,7113.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3574.71,7184.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4812.84,7184.33");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5316.14,7100.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5663.27,7043.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=463.63,6833.30");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=867.88,6784.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1911.45,6767.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4039.37,6781.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4504.14,6925.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5962.01,6833.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1422.76,6594.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2279.83,6628.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2764.81,6654.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3459.91,6707.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4445.44,6541.51");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=803.16,6342.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1827.96,6349.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2479.71,6331.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3133.26,6497.25");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3813.00,6374.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5104.03,6500.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=224.86,6202.71");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1168.17,6043.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1565.32,6023.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2751.43,6081.18");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4138.16,6241.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5614.36,6259.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2269.74,5919.08");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3457.90,5837.74");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3937.00,5950.38");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4574.03,6017.72");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5225.21,5848.92");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=296.29,5646.69");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=747.97,5572.05");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1704.59,5688.02");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2525.21,5679.14");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3066.81,5757.42");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4099.37,5573.11");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4675.45,5608.01");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1043.10,5323.53");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1356.27,5496.38");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2061.25,5401.36");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2983.84,5391.74");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3691.37,5502.38");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5175.54,5430.50");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5784.21,5444.72");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=226.30,5276.37");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=567.25,5038.31");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2585.31,5156.79");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3507.43,5109.89");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3877.95,5187.86");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4722.57,5254.29");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=121.12,4799.69");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1090.04,4782.82");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1584.63,4903.64");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2019.10,4838.00");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2843.35,4854.34");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4166.78,4964.90");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5155.97,4812.48");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5500.53,4932.85");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2352.25,4558.14");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3218.34,4536.11");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3607.45,4548.82");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3972.83,4635.07");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4619.32,4627.23");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=485.83,4317.57");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2790.23,4508.33");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4992.98,4297.41");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5367.06,4317.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5703.30,4415.88");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=25.34,4286.42");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=921.66,4052.20");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1237.93,4224.70");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1781.75,4274.91");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3269.16,4187.74");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3737.66,4119.38");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4539.45,4284.87");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=52.83,3871.06");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1466.44,3883.89");
    }

}

