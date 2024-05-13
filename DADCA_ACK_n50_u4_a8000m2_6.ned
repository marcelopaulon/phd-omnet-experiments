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

network DADCA_ACK_n50_u4_a8000m2_6
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
        @display("i=misc/sensor;p=1569.66,7806.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2788.19,7839.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3194.23,7902.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1218.56,7712.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3638.58,7706.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3959.37,7509.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4402.01,7639.24");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4944.37,7614.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5561.18,7605.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1730.73,7322.96");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2208.16,7485.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2647.59,7344.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3395.10,7451.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4501.87,7294.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1375.65,7247.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3186.42,7110.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3908.41,7130.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4891.61,7252.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5507.55,7214.46");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5867.40,7089.58");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=422.03,6956.36");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=951.26,6946.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1829.98,6774.77");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2500.58,6887.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2938.57,6778.42");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4388.22,6796.34");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5027.21,6828.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=65.20,6558.71");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1493.01,6538.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3291.48,6663.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3753.94,6755.60");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5309.36,6588.22");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5771.55,6651.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=427.84,6510.88");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=977.59,6432.22");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2307.12,6366.46");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2875.85,6369.63");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3758.98,6273.74");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1266.08,6157.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1776.72,6072.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3281.81,6139.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4363.73,6138.06");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5735.30,6033.82");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=501.84,5846.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2366.38,5787.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2784.85,5857.14");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3962.92,5841.52");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4797.37,5930.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5162.99,5818.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=104.69,5701.60");
    }

}

