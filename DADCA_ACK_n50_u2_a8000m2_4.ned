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

network DADCA_ACK_n50_u2_a8000m2_4
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
        @display("i=misc/sensor;p=1049.71,7808.19");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1486.26,7759.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1916.77,7823.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3663.35,7795.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=717.95,7563.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2521.53,7670.74");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3180.03,7616.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4118.69,7642.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1081.84,7404.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1609.43,7372.33");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2073.53,7412.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3702.92,7342.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=20.38,7196.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=974.11,7070.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1911.81,7053.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2483.54,7010.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4155.12,7212.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4805.54,7216.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5504.64,7092.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=555.64,6894.93");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1447.23,6988.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2850.18,6831.56");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4644.27,6889.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5315.23,6787.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=83.25,6628.14");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=514.05,6540.20");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1102.30,6622.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1664.48,6649.22");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2373.32,6573.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3660.63,6682.86");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4308.85,6755.75");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4951.12,6717.80");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5877.23,6748.68");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=128.65,6274.35");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1971.76,6455.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3155.00,6511.65");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4155.20,6408.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4609.16,6493.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5011.46,6362.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1052.54,6121.96");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2754.14,6263.20");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3790.22,6085.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5368.22,6040.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=485.91,5876.18");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1589.53,5937.39");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2221.83,5910.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2571.65,5896.88");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2953.52,5786.21");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3268.38,5964.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4448.10,5869.60");
    }

}

