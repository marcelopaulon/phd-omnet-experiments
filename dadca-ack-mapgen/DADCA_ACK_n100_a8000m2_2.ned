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

network DADCA_ACK_n100_a8000m2_2
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
        @display("i=misc/sensor;p=214.89,7962.61");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=309.29,7972.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=492.19,7991.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=581.68,7978.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2.68,7944.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=67.69,7916.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=127.94,7939.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=424.47,7952.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=644.01,7945.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=212.11,7883.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=345.47,7902.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=504.57,7909.60");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=583.40,7877.93");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=684.95,7892.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=747.60,7881.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=41.86,7857.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=160.36,7835.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=266.63,7835.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=324.44,7796.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=395.92,7813.16");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=461.66,7797.25");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=547.62,7812.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=39.52,7773.89");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=219.96,7780.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=632.83,7773.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=713.69,7774.58");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=780.67,7781.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=892.11,7747.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=142.27,7718.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=299.68,7730.96");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=389.97,7704.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=464.12,7730.83");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=593.93,7715.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=806.11,7721.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=967.27,7709.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=9.58,7655.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=250.41,7670.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=337.55,7671.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=506.21,7677.37");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=687.55,7689.42");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=109.96,7640.65");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=467.08,7628.11");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=651.67,7615.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=802.33,7654.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=875.05,7649.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=931.92,7619.31");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=996.89,7650.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=88.78,7571.45");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=186.21,7589.34");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=258.88,7590.32");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=371.59,7607.94");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=582.86,7598.05");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=717.17,7576.34");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=780.49,7591.02");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4.44,7548.80");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=144.24,7544.60");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=523.83,7549.97");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=936.08,7554.89");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=997.36,7548.57");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=192.71,7496.82");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=257.87,7514.28");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=366.57,7491.83");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=453.85,7525.07");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=573.70,7499.18");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=658.22,7519.21");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=744.70,7491.85");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=829.31,7519.19");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=897.73,7498.10");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=39.03,7475.87");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=104.37,7481.27");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=306.34,7439.96");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=666.56,7451.05");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=962.62,7476.29");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=36.37,7410.64");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=158.17,7424.80");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=446.49,7434.04");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=594.46,7414.64");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=721.47,7424.11");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=816.61,7431.47");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=918.63,7412.41");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=244.63,7394.35");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=410.37,7362.03");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=531.13,7393.37");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=35.37,7342.19");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=120.36,7340.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=521.32,7320.64");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=673.11,7346.97");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=768.14,7352.00");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=867.80,7338.53");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=13.93,7277.06");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=80.86,7281.37");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=184.44,7291.79");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=280.08,7292.68");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=368.01,7280.99");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=447.83,7295.99");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=595.69,7298.80");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=719.77,7294.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=806.21,7297.63");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=938.92,7275.23");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=524.22,7230.93");
    }

}

