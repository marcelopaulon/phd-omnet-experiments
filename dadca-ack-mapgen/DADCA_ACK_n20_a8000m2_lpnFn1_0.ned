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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor;p=35.21,59.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=86.18,16.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=178.41,143.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=137.76,196.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=215.69,255.50");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=287.30,308.31");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=352.49,241.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=416.63,278.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=419.30,356.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=299.02,421.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=374.75,403.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=414.25,484.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=354.61,492.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=501.08,506.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=556.48,582.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=626.95,590.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=690.00,648.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=673.39,736.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=670.26,821.01");
    }

}

