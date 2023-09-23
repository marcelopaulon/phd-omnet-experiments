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

network DADCA_ACK_n50_u2_a8000m2_5
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
        @display("i=misc/sensor;p=790.93,7965.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1347.91,7847.30");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1896.70,7888.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2325.84,7833.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3760.62,7864.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5022.40,7809.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5565.80,7796.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=246.33,7691.50");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=688.00,7594.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1626.56,7560.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2962.95,7706.25");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4497.78,7699.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5822.27,7546.58");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1121.43,7388.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2676.08,7438.08");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3235.74,7466.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3936.39,7498.55");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4383.27,7344.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5405.51,7482.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=663.53,7101.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1165.66,7026.32");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1780.92,7037.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2120.15,7245.06");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3473.12,7201.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4825.12,7249.30");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5313.61,7137.44");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3129.97,6947.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3695.36,6874.30");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4034.17,6974.27");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4398.31,6965.83");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4802.24,6872.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5701.36,6997.99");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=460.24,6734.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1924.60,6693.96");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2607.74,6687.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4476.12,6616.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5033.83,6577.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5682.52,6554.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=204.03,6361.77");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1389.13,6482.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2402.54,6323.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3301.86,6362.32");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3799.48,6481.53");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4144.49,6396.93");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=835.86,6221.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2012.46,6020.31");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2934.29,6227.08");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4743.98,6106.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5403.12,6132.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5954.90,6128.51");
    }

}

