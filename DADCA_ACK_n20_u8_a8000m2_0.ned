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

network DADCA_ACK_n20_u8_a8000m2_0
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
        @display("i=misc/sensor;p=767.65,7932.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1288.26,7819.82");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1678.20,7910.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2660.40,7775.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3579.87,7879.50");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4021.04,7930.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4430.10,7960.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4923.89,7946.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=177.31,7637.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=984.78,7582.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1875.82,7547.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2213.84,7700.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3215.44,7747.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3932.56,7550.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4478.75,7584.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5541.73,7642.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=553.77,7314.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1562.00,7312.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2501.19,7427.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2893.17,7438.20");
    }

}

