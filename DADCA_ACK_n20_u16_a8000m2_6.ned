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

network DADCA_ACK_n20_u16_a8000m2_6
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
        @display("i=misc/sensor;p=1046.59,7814.11");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2372.55,7820.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2973.41,7938.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4624.85,7920.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5401.58,7900.42");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=556.39,7510.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1738.78,7744.59");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2799.88,7595.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3432.75,7613.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4037.38,7608.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1208.70,7346.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1822.27,7316.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2329.33,7456.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4565.55,7468.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4938.42,7380.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5768.50,7482.40");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=237.80,7179.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=713.76,7103.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2444.61,7105.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3221.86,7166.96");
    }

}

