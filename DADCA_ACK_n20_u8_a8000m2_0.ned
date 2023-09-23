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
        @display("i=misc/sensor;p=1015.37,7990.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1351.26,7832.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1780.37,7893.14");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2139.66,7786.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2841.17,7870.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5106.85,7770.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=821.77,7638.16");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1734.26,7542.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3370.12,7749.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4026.02,7689.03");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4413.67,7743.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5587.36,7733.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1344.13,7428.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2329.60,7371.03");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2732.29,7461.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3581.25,7455.48");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4591.74,7315.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5218.17,7303.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5945.43,7385.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=167.26,7109.08");
    }

}

