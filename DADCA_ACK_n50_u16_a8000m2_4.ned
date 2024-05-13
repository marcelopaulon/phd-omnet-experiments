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

network DADCA_ACK_n50_u16_a8000m2_4
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
        @display("i=misc/sensor;p=111.70,7930.24");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=819.65,7759.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2705.74,7811.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3650.95,7876.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4045.38,7975.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4567.32,7948.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4964.50,7933.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5983.09,7894.30");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=472.40,7601.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1429.83,7726.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2198.68,7714.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5533.16,7636.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=132.73,7437.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=778.70,7365.98");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1302.33,7318.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1765.34,7501.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2437.32,7380.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3042.84,7477.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3511.88,7359.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3952.60,7416.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4314.42,7387.60");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4833.07,7487.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5886.15,7499.30");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3744.63,7070.48");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4747.33,7094.39");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5106.84,7214.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5545.49,7132.40");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=392.99,6896.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=806.18,6834.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1184.93,6851.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1554.84,6958.08");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2703.55,6883.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3279.62,6782.56");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4228.66,6871.35");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5238.31,6789.74");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5942.85,6931.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2150.71,6753.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2940.71,6552.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3737.11,6641.63");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4954.32,6533.78");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5640.91,6571.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=329.04,6501.74");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=735.65,6444.33");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1814.97,6501.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3312.11,6342.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4109.78,6473.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5998.63,6376.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=186.14,6036.77");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1155.71,6202.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2005.41,6188.23");
    }

}

