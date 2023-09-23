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

network DADCA_ACK_n20_u2_a8000m2_6
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
        @display("i=misc/sensor;p=171.15,7942.42");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=623.19,7921.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1729.46,7927.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2132.24,7805.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2580.43,7985.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2877.82,7784.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3465.66,7961.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5300.15,7989.49");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=709.76,7579.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1278.23,7731.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4003.52,7668.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4594.34,7652.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5383.42,7531.30");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=211.02,7366.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=991.06,7286.15");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1936.06,7482.38");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2630.97,7401.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3675.37,7346.24");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4987.92,7474.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=459.32,7116.24");
    }

}

