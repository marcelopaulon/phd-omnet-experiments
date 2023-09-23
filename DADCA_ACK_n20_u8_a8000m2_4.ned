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

network DADCA_ACK_n20_u8_a8000m2_4
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
        @display("i=misc/sensor;p=1135.91,7849.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1492.49,7815.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1837.30,7936.01");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2963.71,7914.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3525.11,7777.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4443.70,7873.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=216.26,7647.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=746.80,7538.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2203.83,7638.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3911.86,7595.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4841.51,7682.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5636.25,7720.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=117.74,7311.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1698.19,7476.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2922.32,7480.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3343.28,7283.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4254.08,7381.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5286.67,7499.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=594.11,7119.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1217.12,7215.81");
    }

}

