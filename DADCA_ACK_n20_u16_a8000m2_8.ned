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

network DADCA_ACK_n20_u16_a8000m2_8
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
        @display("i=misc/sensor;p=1053.37,7793.77");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3494.55,7771.02");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4711.73,7795.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5068.34,7893.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=693.28,7715.07");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1649.04,7646.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2055.00,7573.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3987.60,7740.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4372.90,7551.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5362.36,7570.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=908.37,7414.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2785.31,7502.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3125.13,7369.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4742.07,7443.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=303.45,7184.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1504.49,7144.91");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2316.99,7186.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3508.25,7155.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4044.00,7257.53");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4408.07,7159.44");
    }

}

