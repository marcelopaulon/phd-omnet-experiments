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

network DADCA_ACK_n20_u2_a8000m2_7
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
        @display("i=misc/sensor;p=509.20,7871.92");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1832.17,7908.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3085.59,7792.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3883.08,7825.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4668.64,7925.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5170.40,7893.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1081.47,7534.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1820.27,7541.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2511.98,7653.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3493.19,7513.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4293.73,7587.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5570.73,7621.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5933.30,7562.69");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1462.88,7427.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2872.19,7374.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4948.08,7481.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5348.98,7335.84");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=472.97,7249.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1394.35,7016.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1793.04,7148.24");
    }

}

