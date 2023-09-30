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

network DADCA_ACK_n20_u16_a8000m2_1
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
        @display("i=misc/sensor;p=489.35,7946.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1141.69,7777.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1793.78,7782.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3482.10,7982.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4133.05,7916.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5383.58,7929.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5921.19,7825.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=17.57,7590.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2442.40,7739.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3073.26,7520.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3802.36,7688.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4762.26,7711.00");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=752.18,7314.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1457.69,7470.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2125.09,7439.60");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4109.56,7262.87");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4773.85,7307.02");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5141.92,7479.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5575.16,7460.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2475.18,7132.39");
    }

}

