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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=35.73,65.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=78.11,0.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=134.81,40.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=206.19,104.77");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=262.56,40.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=316.36,67.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=321.95,127.22");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=202.20,8.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=417.75,90.92");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=386.55,161.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=465.33,175.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=552.56,191.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=588.81,101.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=497.11,81.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=631.76,40.90");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=694.26,94.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=727.88,154.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=808.30,132.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=856.36,181.12");
    }

}

