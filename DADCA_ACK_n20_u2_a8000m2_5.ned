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

network DADCA_ACK_n20_u2_a8000m2_5
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
        @display("i=misc/sensor;p=799.93,7825.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1352.41,7926.49");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1899.92,7890.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2459.94,7831.07");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5032.54,7987.26");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5863.17,7978.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=354.26,7539.44");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1392.57,7530.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2776.82,7635.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3596.32,7679.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4183.59,7707.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4729.88,7655.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5411.64,7522.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1796.85,7476.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2200.37,7348.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5709.62,7271.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=815.25,7217.29");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1672.07,7088.62");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2988.34,7205.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3718.19,7166.94");
    }

}

