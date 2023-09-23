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

network DADCA_ACK_n50_u4_a8000m2_5
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
        @display("i=misc/sensor;p=111.00,7884.08");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2499.93,7914.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3147.74,7857.09");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3570.26,7812.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4045.99,7948.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5433.10,7915.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1178.05,7747.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1884.93,7628.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2561.04,7554.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4444.99,7695.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5162.31,7561.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=421.98,7406.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1542.60,7445.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2921.31,7371.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3526.56,7459.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4052.71,7370.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4821.07,7362.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5588.46,7401.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5.37,7122.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=730.61,7106.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2359.28,7097.04");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4419.76,7179.03");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5941.00,7033.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1255.03,6935.65");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1798.47,6987.55");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3877.52,6976.04");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4589.84,6781.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4900.69,6977.63");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=289.97,6578.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=790.63,6736.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1772.03,6564.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2840.87,6724.14");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3280.10,6655.04");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5160.29,6535.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5639.86,6573.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1180.19,6328.47");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2234.09,6504.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2801.74,6343.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3644.12,6494.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4279.82,6508.59");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4702.30,6430.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=441.96,6257.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1992.60,6242.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2458.55,6184.58");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3275.07,6129.10");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4578.84,6057.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4964.65,6079.67");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5392.54,6136.21");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=125.18,5858.35");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=767.84,5989.49");
    }

}

