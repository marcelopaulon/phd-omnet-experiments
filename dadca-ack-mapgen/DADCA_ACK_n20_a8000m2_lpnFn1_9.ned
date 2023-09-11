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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor;p=119.99,24.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=159.83,73.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=149.21,142.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=254.58,25.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=28.44,111.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=176.02,0.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=241.32,174.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=304.93,176.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=308.86,244.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=153.17,210.08");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=228.54,290.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=248.10,359.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=179.56,398.28");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=135.71,354.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=377.61,241.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=419.48,168.16");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=444.87,255.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=498.57,210.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=411.04,317.34");
    }

}

