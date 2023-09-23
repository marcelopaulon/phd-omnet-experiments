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

network DADCA_ACK_n50_u3_a8000m2_0
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
        @display("i=misc/sensor;p=4956.94,7755.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=5476.70,7941.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5877.02,7752.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=425.53,7712.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=837.05,7636.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1367.31,7581.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2153.31,7647.23");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2780.31,7631.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3211.17,7530.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3995.52,7709.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4616.07,7521.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=32.47,7405.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=699.63,7283.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4998.07,7356.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5422.62,7334.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5807.67,7383.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=346.68,7046.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1101.03,7054.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1501.90,7021.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1993.95,7012.84");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2305.69,7191.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2761.13,7066.54");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3872.95,7193.64");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4727.08,7115.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=38.88,6845.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3118.82,6932.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4407.58,6954.74");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5778.16,6997.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=549.92,6626.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=945.67,6678.26");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1591.96,6638.53");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2511.95,6656.53");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3467.53,6607.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4041.82,6583.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4945.30,6758.12");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5910.96,6606.38");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=777.91,6325.03");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1288.78,6453.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1910.37,6325.53");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2770.06,6385.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4599.41,6485.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5319.68,6410.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=122.43,6121.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2263.31,6189.48");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3188.82,6081.29");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=581.32,5874.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1299.40,5867.25");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1792.24,5980.60");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2853.22,5854.28");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3544.37,5941.99");
    }

}

